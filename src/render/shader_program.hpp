#pragma once

// Builds a GL program from the game's single-file shader format: one source with
// "#ifdef VERTEX" and "#ifdef FRAGMENT" sections, GLSL ES 1.0 syntax.

#include "render/gl.hpp"

#include <string>
#include <string_view>

namespace sh {

class ShaderProgram {
public:
    ShaderProgram() = default;
    // Throws std::runtime_error with the compiler log on failure.
    ShaderProgram(std::string_view source, std::string_view debug_name);
    ~ShaderProgram();
    ShaderProgram(ShaderProgram&& o) noexcept;
    ShaderProgram& operator=(ShaderProgram&& o) noexcept;
    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    gl::GLuint id() const { return program_; }
    gl::GLint attrib(const char* name) const { return gl::GetAttribLocation(program_, name); }
    gl::GLint uniform(const char* name) const { return gl::GetUniformLocation(program_, name); }

    // Preamble prepended to each stage. Desktop GL compatibility contexts use GLSL 1.20,
    // which has no precision qualifiers, so they are defined away.
    static std::string stage_preamble(bool vertex);

private:
    gl::GLuint program_ = 0;
};

}  // namespace sh
