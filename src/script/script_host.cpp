#include "script/script_host.hpp"

extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}

#include <cmath>
#include <stdexcept>

namespace sh::script {

float ScriptArgs::as_float(int i) const {
    if (count_ <= i) return 0.0f;
    return static_cast<float>(lua_tonumber(L_, i + 1));
}

int ScriptArgs::as_int(int i) const {
    if (count_ <= i) return 0;
    return static_cast<int>(lua_tointeger(L_, i + 1));
}

bool ScriptArgs::as_bool(int i) const {
    if (count_ <= i) return false;
    return lua_toboolean(L_, i + 1) == 1;
}

std::string ScriptArgs::as_string(int i) const {
    if (count_ <= i) return {};
    const char* s = lua_tolstring(L_, i + 1, nullptr);
    return s ? std::string(s) : std::string();
}

void ScriptRet::add_float(float v) {
    lua_pushnumber(L_, static_cast<double>(v));
    ++count_;
}

void ScriptRet::add_int(int v) {
    lua_pushinteger(L_, v);
    ++count_;
}

void ScriptRet::add_bool(bool v) {
    lua_pushboolean(L_, v ? 1 : 0);
    ++count_;
}

void ScriptRet::add_string(const std::string& v) {
    lua_pushstring(L_, v.c_str());
    ++count_;
}

namespace {

// scriptProxy: upvalue 1 is the registered function, upvalue 2 the host.
int proxy(lua_State* L) {
    auto* fn = static_cast<ScriptHost::Fn*>(lua_touserdata(L, lua_upvalueindex(1)));
    ScriptArgs args(L, lua_gettop(L));
    ScriptRet ret(L);
    (*fn)(args, ret);
    return ret.count();
}

ScriptHost* host_of(lua_State* L) { return static_cast<ScriptHost*>(lua_touserdata(L, lua_upvalueindex(1))); }

// Lua 5.1.4's math_random with bionic's rand(), which is lrand48 (RAND_MAX 0x7fffffff). The draw comes first,
// even when the arguments turn out to be wrong.
int math_random(lua_State* L) {
    const double r = static_cast<double>(host_of(L)->rng().lrand48() % 0x7fffffff) / 2147483647.0;
    switch (lua_gettop(L)) {
        case 0:
            lua_pushnumber(L, r);
            break;
        case 1: {
            const int u = luaL_checkint(L, 1);
            luaL_argcheck(L, 1 <= u, 1, "interval is empty");
            lua_pushnumber(L, std::floor(r * u) + 1.0);
            break;
        }
        case 2: {
            const int l = luaL_checkint(L, 1);
            const int u = luaL_checkint(L, 2);
            luaL_argcheck(L, l <= u, 2, "interval is empty");
            lua_pushnumber(L, std::floor(r * (u - l + 1)) + l);
            break;
        }
        default:
            return luaL_error(L, "wrong number of arguments");
    }
    return 1;
}

int math_randomseed(lua_State* L) {
    host_of(L)->rng().srand48(luaL_checkint(L, 1));
    return 0;
}

int panic(lua_State* L) {
    const char* msg = lua_tostring(L, -1);
    throw std::runtime_error(std::string("Lua panic: ") + (msg ? msg : "?"));
}

}  // namespace

ScriptHost::ScriptHost(Rand48& rng) : rng_(&rng) {
    L_ = luaL_newstate();
    if (!L_) throw std::runtime_error("cannot create a Lua state");
    lua_atpanic(L_, panic);
    luaL_openlibs(L_);
    lua_getglobal(L_, "math");
    lua_pushlightuserdata(L_, this);
    lua_pushcclosure(L_, math_random, 1);
    lua_setfield(L_, -2, "random");
    lua_pushlightuserdata(L_, this);
    lua_pushcclosure(L_, math_randomseed, 1);
    lua_setfield(L_, -2, "randomseed");
    lua_pop(L_, 1);
}

ScriptHost::~ScriptHost() {
    if (L_) lua_close(L_);
}

void ScriptHost::register_function(const std::string& name, Fn fn) {
    functions_.push_back(std::make_unique<Fn>(std::move(fn)));
    lua_pushlightuserdata(L_, functions_.back().get());
    lua_pushlightuserdata(L_, this);
    lua_pushcclosure(L_, proxy, 2);
    lua_setglobal(L_, name.c_str());
}

bool ScriptHost::load(const std::string& chunk_name, std::string_view source) {
    if (luaL_loadbuffer(L_, source.data(), source.size(), chunk_name.c_str()) == 0 && lua_pcall(L_, 0, 0, 0) == 0)
        return true;
    const char* msg = lua_tolstring(L_, -1, nullptr);
    if (on_error) on_error(msg ? msg : "?");
    lua_settop(L_, -2);
    return false;
}

bool ScriptHost::execute(const std::string& code) {
    if (luaL_loadstring(L_, code.c_str()) == 0 && lua_pcall(L_, 0, LUA_MULTRET, 0) == 0) return true;
    const char* msg = lua_tolstring(L_, -1, nullptr);
    if (on_error) on_error(msg ? msg : "?");
    lua_settop(L_, -2);
    return false;
}

bool ScriptHost::call(const std::string& name) {
    lua_getglobal(L_, name.c_str());
    if (lua_type(L_, -1) != LUA_TFUNCTION) {
        lua_settop(L_, 0);
        return false;
    }
    return lua_pcall(L_, 0, 0, 0) == 0;
}

bool ScriptHost::call(const std::string& name, const std::string& arg) {
    lua_getglobal(L_, name.c_str());
    if (lua_type(L_, -1) != LUA_TFUNCTION) {
        lua_settop(L_, 0);
        return false;
    }
    lua_pushstring(L_, arg.c_str());
    return lua_pcall(L_, 1, 0, 0) == 0;
}

bool ScriptHost::has_function(const std::string& name) {
    lua_getglobal(L_, name.c_str());
    const bool f = lua_type(L_, -1) == LUA_TFUNCTION;
    lua_settop(L_, -2);
    return f;
}

void ScriptHost::collect_garbage(int n) {
    if (n > 0) lua_gc(L_, LUA_GCSTEP, n);
    else lua_gc(L_, LUA_GCCOLLECT, 0);
}

}  // namespace sh::script
