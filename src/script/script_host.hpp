#pragma once

// The Lua host, after the original's QiScript: one Lua 5.1.4 state with the standard
// libraries, native functions registered as globals, global functions called by name. math.random and
// math.randomseed draw from the shared lrand48 stream as on the device (bionic's rand is lrand48).

#include "core/rand48.hpp"

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

struct lua_State;

namespace sh::script {

// QiScriptArgs: argument i (0-based) of a native call. Missing arguments read as 0, false or "".
class ScriptArgs {
public:
    ScriptArgs(lua_State* L, int count) : L_(L), count_(count) {}
    int size() const { return count_; }
    float as_float(int i) const;       // lua_tonumber narrowed to float
    int as_int(int i) const;           // lua_tointeger (truncates)
    bool as_bool(int i) const;         // lua_toboolean == 1
    std::string as_string(int i) const;  // lua_tolstring ("" for nil)
    int as_handle(int i) const { return as_int(i); }  // the original passes object pointers as integers

private:
    lua_State* L_;
    int count_;
};

// QiScriptRet: values pushed as results; the count is what the native call returns to Lua.
class ScriptRet {
public:
    explicit ScriptRet(lua_State* L) : L_(L) {}
    void add_float(float v);   // widened to double
    void add_int(int v);
    void add_bool(bool v);
    void add_string(const std::string& v);
    void add_handle(int v) { add_int(v); }
    int count() const { return count_; }

private:
    lua_State* L_;
    int count_ = 0;
};

class ScriptHost {
public:
    using Fn = std::function<void(const ScriptArgs&, ScriptRet&)>;

    explicit ScriptHost(Rand48& rng);
    ~ScriptHost();
    ScriptHost(const ScriptHost&) = delete;
    ScriptHost& operator=(const ScriptHost&) = delete;

    // QiScript::registerFunction: a global of that name calling fn.
    void register_function(const std::string& name, Fn fn);
    // QiScript::load: compile and run a chunk. On failure the message goes to on_error and false is returned.
    bool load(const std::string& chunk_name, std::string_view source);
    // QiScript::execute: run a string.
    bool execute(const std::string& code);
    // QiScript::call: call a global function with no argument or one string. False if it is not a function
    // (the stack is then cleared) or if it fails (as in the original, the error message stays on the stack).
    bool call(const std::string& name);
    bool call(const std::string& name, const std::string& arg);
    bool has_function(const std::string& name);
    // QiScript::collectGarbage: n > 0 runs a GC step of size n, otherwise a full collection.
    void collect_garbage(int n);

    lua_State* state() const { return L_; }
    Rand48& rng() const { return *rng_; }
    std::function<void(const std::string&)> on_error;  // QiScript::print of error messages

private:
    lua_State* L_ = nullptr;
    Rand48* rng_;
    std::vector<std::unique_ptr<Fn>> functions_;
};

}  // namespace sh::script
