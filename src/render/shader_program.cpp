#include "render/shader_program.hpp"

#include <stdexcept>
#include <utility>

namespace sh {

namespace {

gl::GLuint compile(gl::GLenum stage, const std::string& source, std::string_view debug_name) {
    const gl::GLuint s = gl::CreateShader(stage);
    const char* text = source.c_str();
    gl::ShaderSource(s, 1, &text, nullptr);
    gl::CompileShader(s);
    gl::GLint ok = 0;
    gl::GetShaderiv(s, gl::COMPILE_STATUS, &ok);
    if (!ok) {
        gl::GLint len = 0;
        gl::GetShaderiv(s, gl::INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 1 ? len : 1), '\0');
        gl::GetShaderInfoLog(s, len, nullptr, log.data());
        gl::DeleteShader(s);
        throw std::runtime_error(std::string(debug_name) + (stage == gl::VERTEX_SHADER ? " (vertex): " : " (fragment): ") +
                                 log);
    }
    return s;
}

}  // namespace

std::string ShaderProgram::stage_preamble(bool vertex) {
    return std::string("#version 120\n#define lowp\n#define mediump\n#define highp\n") +
           (vertex ? "#define VERTEX\n" : "#define FRAGMENT\n") + "#line 1\n";
}

ShaderProgram::ShaderProgram(std::string_view source, std::string_view debug_name) {
    const std::string src(source);
    const gl::GLuint vs = compile(gl::VERTEX_SHADER, stage_preamble(true) + src, debug_name);
    gl::GLuint fs = 0;
    try {
        fs = compile(gl::FRAGMENT_SHADER, stage_preamble(false) + src, debug_name);
    } catch (...) {
        gl::DeleteShader(vs);
        throw;
    }
    program_ = gl::CreateProgram();
    gl::AttachShader(program_, vs);
    gl::AttachShader(program_, fs);
    gl::LinkProgram(program_);
    gl::DeleteShader(vs);
    gl::DeleteShader(fs);
    gl::GLint ok = 0;
    gl::GetProgramiv(program_, gl::LINK_STATUS, &ok);
    if (!ok) {
        gl::GLint len = 0;
        gl::GetProgramiv(program_, gl::INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 1 ? len : 1), '\0');
        gl::GetProgramInfoLog(program_, len, nullptr, log.data());
        gl::DeleteProgram(program_);
        program_ = 0;
        throw std::runtime_error(std::string(debug_name) + " (link): " + log);
    }
}

ShaderProgram::~ShaderProgram() {
    if (program_) gl::DeleteProgram(program_);
}

ShaderProgram::ShaderProgram(ShaderProgram&& o) noexcept : program_(std::exchange(o.program_, 0)) {}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& o) noexcept {
    if (this != &o) {
        if (program_) gl::DeleteProgram(program_);
        program_ = std::exchange(o.program_, 0);
    }
    return *this;
}

}  // namespace sh
