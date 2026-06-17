#include "Mesh.hpp"
#include <cstring>

// Interleaved vertex layout: [ pos.x pos.y pos.z  norm.x norm.y norm.z ]
//   location 0 = position (3 floats)
//   location 1 = normal   (3 floats)

Mesh::~Mesh() { release(); }

Mesh::Mesh(Mesh&& o) noexcept
    : m_vao(o.m_vao), m_vbo(o.m_vbo), m_ebo(o.m_ebo), m_count(o.m_count)
{
    o.m_vao = o.m_vbo = o.m_ebo = 0;
    o.m_count = 0;
}

Mesh& Mesh::operator=(Mesh&& o) noexcept {
    if(this != &o) {
        release();
        m_vao = o.m_vao; m_vbo = o.m_vbo; m_ebo = o.m_ebo; m_count = o.m_count;
        o.m_vao = o.m_vbo = o.m_ebo = 0;
        o.m_count = 0;
    }
    return *this;
}

void Mesh::release() {
    if(m_vao) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    if(m_vbo) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if(m_ebo) { glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
    m_count = 0;
}

void Mesh::upload(const STLMesh& stl) {
    release();
    if(stl.vertices.empty()) return;

    size_t vCount = stl.vertices.size();
    std::vector<float> vdata;
    vdata.reserve(vCount * 6);
    for(size_t i = 0; i < vCount; ++i) {
        vdata.push_back(stl.vertices[i].x);
        vdata.push_back(stl.vertices[i].y);
        vdata.push_back(stl.vertices[i].z);
        vdata.push_back(stl.normals[i].x);
        vdata.push_back(stl.normals[i].y);
        vdata.push_back(stl.normals[i].z);
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vdata.size() * sizeof(float)),
                 vdata.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(stl.indices.size() * sizeof(uint32_t)),
                 stl.indices.data(), GL_STATIC_DRAW);

    constexpr GLsizei stride = 6 * sizeof(float);
    // position (location 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(0));
    // normal (location 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);

    m_count = static_cast<GLsizei>(stl.indices.size());
}

void Mesh::draw() const {
    if(!m_vao) return;
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_count, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}
