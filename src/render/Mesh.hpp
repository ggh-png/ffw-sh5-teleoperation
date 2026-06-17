#pragma once
#include "io/STLLoader.hpp"
#include <epoxy/gl.h>
#include <vector>

// GPU-side mesh: VAO + VBO (interleaved pos+normal) + EBO
class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    // Non-copyable (owns GPU resources)
    Mesh(const Mesh&)            = delete;
    Mesh& operator=(const Mesh&) = delete;

    // Movable
    Mesh(Mesh&& o) noexcept;
    Mesh& operator=(Mesh&& o) noexcept;

    // Upload STL data to GPU
    void upload(const STLMesh& stl);

    void draw() const;

    bool valid() const { return m_vao != 0; }
    int  triangleCount() const { return m_count / 3; }

private:
    GLuint m_vao   = 0;
    GLuint m_vbo   = 0;
    GLuint m_ebo   = 0;
    GLsizei m_count = 0; // number of indices

    void release();
};
