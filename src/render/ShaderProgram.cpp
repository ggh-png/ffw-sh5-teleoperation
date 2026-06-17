#include "ShaderProgram.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdio>

ShaderProgram::~ShaderProgram() {
    if(m_id) glDeleteProgram(m_id);
}

std::string ShaderProgram::readFile(const std::string& path) {
    std::ifstream f(path);
    if(!f.is_open()) throw std::runtime_error("ShaderProgram: cannot open " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

GLuint ShaderProgram::compileShader(GLenum type, const std::string& src) {
    GLuint shader = glCreateShader(type);
    const char* c = src.c_str();
    glShaderSource(shader, 1, &c, nullptr);
    glCompileShader(shader);

    GLint ok;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if(!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("Shader compile error:\n") + log);
    }
    return shader;
}

bool ShaderProgram::loadFiles(const std::string& vertPath,
                              const std::string& fragPath) {
    return loadSources(readFile(vertPath), readFile(fragPath));
}

bool ShaderProgram::loadSources(const std::string& vertSrc,
                                const std::string& fragSrc) {
    if(m_id) { glDeleteProgram(m_id); m_id = 0; }

    GLuint vert = compileShader(GL_VERTEX_SHADER,   vertSrc);
    GLuint frag = compileShader(GL_FRAGMENT_SHADER, fragSrc);

    m_id = glCreateProgram();
    glAttachShader(m_id, vert);
    glAttachShader(m_id, frag);
    glLinkProgram(m_id);

    glDeleteShader(vert);
    glDeleteShader(frag);

    GLint ok;
    glGetProgramiv(m_id, GL_LINK_STATUS, &ok);
    if(!ok) {
        char log[1024];
        glGetProgramInfoLog(m_id, 1024, nullptr, log);
        glDeleteProgram(m_id);
        m_id = 0;
        throw std::runtime_error(std::string("Shader link error:\n") + log);
    }
    return true;
}

void ShaderProgram::use()   const { glUseProgram(m_id); }
void ShaderProgram::unuse() const { glUseProgram(0); }

void ShaderProgram::setInt(const char* n, int v) const {
    glUniform1i(glGetUniformLocation(m_id, n), v);
}
void ShaderProgram::setFloat(const char* n, float v) const {
    glUniform1f(glGetUniformLocation(m_id, n), v);
}
void ShaderProgram::setVec3(const char* n, const Vec3& v) const {
    glUniform3f(glGetUniformLocation(m_id, n), v.x, v.y, v.z);
}
void ShaderProgram::setVec4(const char* n, const Vec4& v) const {
    glUniform4f(glGetUniformLocation(m_id, n), v.x, v.y, v.z, v.w);
}
void ShaderProgram::setMat4(const char* n, const Mat4& m) const {
    glUniformMatrix4fv(glGetUniformLocation(m_id, n), 1, GL_FALSE, m.ptr());
}
