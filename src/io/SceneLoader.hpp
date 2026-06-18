#pragma once
#include "math/Math.hpp"
#include <string>
#include <vector>

// Describes one physics + render object loaded from a scene XML file.
struct ObjectDesc {
    enum class Type { Box, Cylinder } type = Type::Box;
    std::string name;
    Vec3  pos         = {};
    Vec3  halfExtents = {};      // Box
    float radius      = 0.f;    // Cylinder
    float halfHeight  = 0.f;    // Cylinder
    float mass        = 0.f;    // 0 = static, > 0 = dynamic
    float friction    = 0.5f;
    Vec3  color       = {0.8f, 0.8f, 0.8f};
};

// Parses a scene XML file in the following format (Y-up physics coordinates):
//
//   <scene>
//     <body name="table" pos="0 0.325 -0.65">
//       <geom type="box" size="0.30 0.325 0.25"
//             mass="0" friction="0.8" rgba="0.55 0.35 0.15 1"/>
//     </body>
//     <body name="can" pos="0 0.71 -0.65">
//       <geom type="cylinder" size="0.033 0.06"
//             mass="0.35" friction="0.5" rgba="0.85 0.08 0.08 1"/>
//     </body>
//   </scene>
//
// box size = "half_x half_y half_z"   (MJCF convention)
// cylinder size = "radius half_height" (MJCF convention)
// mass=0 → static rigid body; mass>0 → dynamic rigid body
class SceneLoader {
public:
    static std::vector<ObjectDesc> load(const std::string& xmlPath);
};
