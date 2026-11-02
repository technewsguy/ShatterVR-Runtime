#include "render/gl.hpp"

namespace sh::gl {

#define SH_GL_DEFINE(ret, name, params) ret(SH_GLAPI* name) params = nullptr;
SH_GL_FUNCTIONS(SH_GL_DEFINE)
#undef SH_GL_DEFINE

const char* load(GetProcFn get_proc) {
#define SH_GL_LOAD(ret, name, params)                                      \
    name = reinterpret_cast<ret(SH_GLAPI*) params>(get_proc("gl" #name)); \
    if (!name) return "gl" #name;
    SH_GL_FUNCTIONS(SH_GL_LOAD)
#undef SH_GL_LOAD
    return nullptr;
}

}  // namespace sh::gl
