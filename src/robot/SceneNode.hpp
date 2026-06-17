#pragma once
#include "Joint.hpp"
#include "math/Math.hpp"
#include <string>
#include <vector>
#include <memory>

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
    int meshIndex = -1;

    // Color tint for visualization (0-1 RGB)
    Vec3 color = {0.7f, 0.7f, 0.75f};

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
