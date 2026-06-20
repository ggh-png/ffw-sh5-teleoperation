#pragma once
#include "math/Vec3.hpp"
#include <vector>
#include <string>
#include <cstdint>

struct STLMesh {
    std::vector<Vec3>     vertices; // 3 per triangle, unindexed
    std::vector<Vec3>     normals;  // 1 per vertex (flat shading = same 3 per tri)
    std::vector<uint32_t> indices;
    std::vector<float>    uvs;      // optional: 2 floats (u,v) per vertex; empty = no UVs

    bool empty() const { return vertices.empty(); }
};

// Binary STL loader.
// Format: [80B header][4B triCount][ triCount × 50B triangle ]
// Triangle: [12B normal][12B v0][12B v1][12B v2][2B attr]
class STLLoader {
public:
    static STLMesh load(const std::string& path);
};
