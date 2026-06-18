#pragma once
#include "Joint.hpp"
#include "math/Math.hpp"
#include <string>
#include <vector>
#include <memory>

// Inertial properties from MJCF <inertial> element.
// Used to configure Bullet rigid-body mass/inertia for dynamic simulation.
struct Inertial {
    float mass        = 0.f;            // kg
    Vec3  com         = {};             // centre of mass in body frame [m]
    Vec3  diagInertia = {};             // principal moments [kg·m²]
    Quaternion frame  = Quaternion::identity(); // principal-axes orientation
    bool  valid       = false;          // true only when parsed from MJCF
};

struct SceneNode {
    std::string name;

    // Fixed body-frame offset relative to parent body (from MJCF body pos/quat)
    Vec3       localPos  = {0,0,0};
    Quaternion localRot  = Quaternion::identity();
    Vec3       localScale = {1,1,1};

    // Variable joint connecting this body to its parent
    Joint joint;

    // Computed by ForwardKinematics each frame
    Mat4 worldTransform = Mat4::identity();

    // Rendering: index into RobotModel::meshes (-1 = no mesh)
    int  meshIndex = -1;
    Vec3 meshScale = {1,1,1}; // per-mesh scale (e.g. 0.001 for mm→m STLs)

    // Color tint for visualization (0-1 RGB)
    Vec3 color = {0.7f, 0.7f, 0.75f};

    // ── Physics properties (from MJCF <inertial> / collision <geom>) ───────
    Inertial inertial;           // body mass/inertia from <inertial>
    float    geomFriction = 0.7f; // sliding friction of first collision geom
    float    geomRadius   = 0.f;  // cylinder/sphere collision radius (0 = mesh/box only)

    // Scene graph
    SceneNode*                            parent   = nullptr;
    std::vector<std::unique_ptr<SceneNode>> children;

    SceneNode* addChild(std::unique_ptr<SceneNode> child) {
        child->parent = this;
        children.push_back(std::move(child));
        return children.back().get();
    }

    // Fixed body-local transform (does NOT include joint DOF)
    Mat4 localTransform() const {
        return Mat4::translation(localPos)
             * Mat4::fromQuaternion(localRot)
             * Mat4::scaling(localScale);
    }
};
