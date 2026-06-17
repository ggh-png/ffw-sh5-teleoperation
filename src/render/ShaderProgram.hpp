#pragma once
#include "math/Math.hpp"
#include <GL/glew.h>
#include <string>

class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    // Compile and link from source file paths
    bool loadFiles(const std::string& vertPath, const std::string& fragPath);

    // Compile and link from source strings
    bool loadSources(const std::string& vertSrc, const std::string& fragSrc);

    void use() const;
    void unuse() const;

    GLuint id() const { return m_id; }
    bool   valid() const { return m_id != 0; }

    // Uniform setters
    void setInt  (const char* name, int v)         const;
    void setFloat(const char* name, float v)       const;
    void setVec3 (const char* name, const Vec3& v) const;
    void setVec4 (const char* name, const Vec4& v) const;
    void setMat4 (const char* name, const Mat4& m) const;

private:
    GLuint m_id = 0;

    static GLuint compileShader(GLenum type, const std::string& src);
    static std::string readFile(const std::string& path);
};
