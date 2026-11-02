#pragma once

// Minimal OpenGL function loader: only what the renderer uses. Kept to the
// GLES 2/3-compatible subset so the same calls work on desktop GL and GLES.
// Functions live in namespace sh::gl and are loaded at runtime from the context.

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
#define SH_GLAPI __stdcall
#else
#define SH_GLAPI
#endif

namespace sh::gl {

using GLenum = unsigned int;
using GLuint = unsigned int;
using GLint = int;
using GLsizei = int;
using GLboolean = unsigned char;
using GLbitfield = unsigned int;
using GLfloat = float;
using GLchar = char;
using GLubyte = unsigned char;
using GLsizeiptr = std::ptrdiff_t;

constexpr GLenum FALSE_ = 0, TRUE_ = 1;
constexpr GLenum NO_ERROR_ = 0;
constexpr GLenum DEPTH_BUFFER_BIT = 0x0100, COLOR_BUFFER_BIT = 0x4000;
constexpr GLenum LINES = 0x0001, TRIANGLES = 0x0004, TRIANGLE_STRIP = 0x0005;
constexpr GLenum LEQUAL = 0x0203, LESS = 0x0201;
constexpr GLenum ZERO = 0, ONE = 1, SRC_ALPHA = 0x0302, ONE_MINUS_SRC_ALPHA = 0x0303;
constexpr GLenum CULL_FACE = 0x0B44, DEPTH_TEST = 0x0B71, BLEND = 0x0BE2, SCISSOR_TEST = 0x0C11;
constexpr GLenum UNPACK_ALIGNMENT = 0x0CF5, PACK_ALIGNMENT = 0x0D05;
constexpr GLenum TEXTURE_2D = 0x0DE1;
constexpr GLenum UNSIGNED_BYTE = 0x1401, UNSIGNED_SHORT = 0x1403, FLOAT = 0x1406;
constexpr GLenum RGB = 0x1907, RGBA = 0x1908;
constexpr GLenum VENDOR = 0x1F00, RENDERER = 0x1F01, VERSION = 0x1F02, SHADING_LANGUAGE_VERSION = 0x8B8C;
constexpr GLenum NEAREST = 0x2600, LINEAR = 0x2601, LINEAR_MIPMAP_LINEAR = 0x2703;
constexpr GLenum TEXTURE_MAG_FILTER = 0x2800, TEXTURE_MIN_FILTER = 0x2801;
constexpr GLenum TEXTURE_WRAP_S = 0x2802, TEXTURE_WRAP_T = 0x2803;
constexpr GLenum REPEAT = 0x2901, CLAMP_TO_EDGE = 0x812F;
constexpr GLenum TEXTURE0 = 0x84C0;
constexpr GLenum ARRAY_BUFFER = 0x8892, ELEMENT_ARRAY_BUFFER = 0x8893, STATIC_DRAW = 0x88E4, STREAM_DRAW = 0x88E0;
constexpr GLenum FRAGMENT_SHADER = 0x8B30, VERTEX_SHADER = 0x8B31;
constexpr GLenum COMPILE_STATUS = 0x8B81, LINK_STATUS = 0x8B82, INFO_LOG_LENGTH = 0x8B84;
constexpr GLenum FRAMEBUFFER = 0x8D40, RENDERBUFFER = 0x8D41, COLOR_ATTACHMENT0 = 0x8CE0;
constexpr GLenum READ_FRAMEBUFFER = 0x8CA8, DRAW_FRAMEBUFFER = 0x8CA9, DEPTH_COMPONENT24 = 0x81A6;
constexpr GLenum FRAMEBUFFER_SRGB = 0x8DB9, SRGB8_ALPHA8 = 0x8C43, RGBA8 = 0x8058;
constexpr GLenum DEPTH_STENCIL_ATTACHMENT = 0x821A, DEPTH24_STENCIL8 = 0x88F0;
constexpr GLenum STENCIL_BUFFER_BIT = 0x0400, STENCIL_TEST = 0x0B90, POLYGON_OFFSET_FILL = 0x8037;
constexpr GLenum EQUAL = 0x0202, ALWAYS = 0x0207, KEEP = 0x1E00, REPLACE = 0x1E01, FRONT = 0x0404, BACK = 0x0405;
constexpr GLenum UNSIGNED_INT = 0x1405;

// X(return type, name, parameter list)
#define SH_GL_FUNCTIONS(X)                                                                          \
    X(void, ClearColor, (GLfloat, GLfloat, GLfloat, GLfloat))                                       \
    X(void, Clear, (GLbitfield))                                                                    \
    X(void, Viewport, (GLint, GLint, GLsizei, GLsizei))                                             \
    X(void, Scissor, (GLint, GLint, GLsizei, GLsizei))                                              \
    X(void, Enable, (GLenum))                                                                       \
    X(void, Disable, (GLenum))                                                                      \
    X(void, DepthFunc, (GLenum))                                                                    \
    X(void, DepthMask, (GLboolean))                                                                 \
    X(void, BlendFunc, (GLenum, GLenum))                                                            \
    X(GLenum, GetError, ())                                                                         \
    X(const GLubyte*, GetString, (GLenum))                                                          \
    X(void, PixelStorei, (GLenum, GLint))                                                           \
    X(void, ReadPixels, (GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, void*))                    \
    X(void, Finish, ())                                                                             \
    X(void, GenBuffers, (GLsizei, GLuint*))                                                         \
    X(void, DeleteBuffers, (GLsizei, const GLuint*))                                                \
    X(void, BindBuffer, (GLenum, GLuint))                                                           \
    X(void, BufferData, (GLenum, GLsizeiptr, const void*, GLenum))                                  \
    X(GLuint, CreateShader, (GLenum))                                                               \
    X(void, DeleteShader, (GLuint))                                                                 \
    X(void, ShaderSource, (GLuint, GLsizei, const GLchar* const*, const GLint*))                    \
    X(void, CompileShader, (GLuint))                                                                \
    X(void, GetShaderiv, (GLuint, GLenum, GLint*))                                                  \
    X(void, GetShaderInfoLog, (GLuint, GLsizei, GLsizei*, GLchar*))                                 \
    X(GLuint, CreateProgram, ())                                                                    \
    X(void, DeleteProgram, (GLuint))                                                                \
    X(void, AttachShader, (GLuint, GLuint))                                                         \
    X(void, LinkProgram, (GLuint))                                                                  \
    X(void, GetProgramiv, (GLuint, GLenum, GLint*))                                                 \
    X(void, GetProgramInfoLog, (GLuint, GLsizei, GLsizei*, GLchar*))                                \
    X(void, UseProgram, (GLuint))                                                                   \
    X(GLint, GetAttribLocation, (GLuint, const GLchar*))                                            \
    X(GLint, GetUniformLocation, (GLuint, const GLchar*))                                           \
    X(void, Uniform1i, (GLint, GLint))                                                              \
    X(void, Uniform4fv, (GLint, GLsizei, const GLfloat*))                                           \
    X(void, UniformMatrix4fv, (GLint, GLsizei, GLboolean, const GLfloat*))                          \
    X(void, EnableVertexAttribArray, (GLuint))                                                      \
    X(void, DisableVertexAttribArray, (GLuint))                                                     \
    X(void, VertexAttribPointer, (GLuint, GLint, GLenum, GLboolean, GLsizei, const void*))          \
    X(void, DrawArrays, (GLenum, GLint, GLsizei))                                                   \
    X(void, DrawElements, (GLenum, GLsizei, GLenum, const void*))                                   \
    X(void, GenTextures, (GLsizei, GLuint*))                                                        \
    X(void, DeleteTextures, (GLsizei, const GLuint*))                                               \
    X(void, BindTexture, (GLenum, GLuint))                                                          \
    X(void, ActiveTexture, (GLenum))                                                                \
    X(void, TexImage2D, (GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void*)) \
    X(void, TexParameteri, (GLenum, GLenum, GLint))                                                 \
    X(void, GenerateMipmap, (GLenum))                                                               \
    X(void, GenFramebuffers, (GLsizei, GLuint*))                                                    \
    X(void, BindFramebuffer, (GLenum, GLuint))                                                      \
    X(void, FramebufferTexture2D, (GLenum, GLenum, GLenum, GLuint, GLint))                          \
    X(void, GenRenderbuffers, (GLsizei, GLuint*))                                                   \
    X(void, BindRenderbuffer, (GLenum, GLuint))                                                     \
    X(void, RenderbufferStorage, (GLenum, GLenum, GLsizei, GLsizei))                                \
    X(void, FramebufferRenderbuffer, (GLenum, GLenum, GLenum, GLuint))                              \
    X(void, DeleteFramebuffers, (GLsizei, const GLuint*))                                           \
    X(void, DeleteRenderbuffers, (GLsizei, const GLuint*))                                          \
    X(void, PolygonOffset, (GLfloat, GLfloat))                                                      \
    X(void, StencilFunc, (GLenum, GLint, GLuint))                                                   \
    X(void, StencilOp, (GLenum, GLenum, GLenum))                                                    \
    X(void, CullFace, (GLenum))                                                                     \
    X(void, LineWidth, (GLfloat))                                                                   \
    X(void, Uniform3fv, (GLint, GLsizei, const GLfloat*))                                           \
    X(void, Uniform2fv, (GLint, GLsizei, const GLfloat*))                                           \
    X(void, ColorMask, (GLboolean, GLboolean, GLboolean, GLboolean))                                \
    X(void, BlitFramebuffer, (GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum))

#define SH_GL_DECLARE(ret, name, params) extern ret(SH_GLAPI* name) params;
SH_GL_FUNCTIONS(SH_GL_DECLARE)
#undef SH_GL_DECLARE

using GetProcFn = void* (*)(const char* name);

// Loads every function above using the context's proc-address lookup.
// Returns the name of the first missing function, or nullptr on success.
const char* load(GetProcFn get_proc);

}  // namespace sh::gl
