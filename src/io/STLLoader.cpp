#include "STLLoader.hpp"
#include <fstream>
#include <cstring>
#include <stdexcept>

static float readF32(const char* buf, size_t& offset) {
    float v;
    std::memcpy(&v, buf + offset, 4);
    offset += 4;
    return v;
}

STLMesh STLLoader::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if(!f.is_open())
        throw std::runtime_error("STLLoader: cannot open " + path);

    std::streamsize size = f.tellg();
    f.seekg(0, std::ios::beg);

    std::vector<char> buf(static_cast<size_t>(size));
    if(!f.read(buf.data(), size))
        throw std::runtime_error("STLLoader: read error " + path);

    if(size < 84)
        throw std::runtime_error("STLLoader: file too small " + path);

    // header: 80 bytes (skip)
    size_t offset = 80;

    uint32_t triCount;
    std::memcpy(&triCount, buf.data() + offset, 4);
    offset += 4;

    STLMesh mesh;
    mesh.vertices.reserve(triCount * 3);
    mesh.normals.reserve(triCount * 3);
    mesh.indices.reserve(triCount * 3);

    for(uint32_t i = 0; i < triCount; ++i) {
        // Normal
        float nx = readF32(buf.data(), offset);
        float ny = readF32(buf.data(), offset);
        float nz = readF32(buf.data(), offset);
        Vec3 normal{nx, ny, nz};

        // 3 vertices
        for(int v = 0; v < 3; ++v) {
            float vx = readF32(buf.data(), offset);
            float vy = readF32(buf.data(), offset);
            float vz = readF32(buf.data(), offset);
            mesh.vertices.push_back({vx, vy, vz});
            mesh.normals.push_back(normal);
            mesh.indices.push_back(static_cast<uint32_t>(mesh.vertices.size() - 1));
        }

        // Attribute byte count (2 bytes, skip)
        offset += 2;
    }

    return mesh;
}
