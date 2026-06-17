#pragma once
#include "math/Math.hpp"
#include <string>

enum class JointType {
    Fixed,      // no DOF, body locked to parent
    Revolute,   // 1 DOF rotation around axis  (MJCF: hinge)
    Prismatic,  // 1 DOF translation along axis (MJCF: slide)
    Free,       // 6 DOF floating base           (MJCF: free)
    Ball,       // 3 DOF rotation                (MJCF: ball)
};

struct Joint {
    std::string name;
    JointType   type  = JointType::Fixed;
    Vec3        axis  = {0, 0, 1};   // rotation / translation axis (local body frame)
    float       limitLo = -kPi;      // rad or m
    float       limitHi =  kPi;
    float       value   = 0.f;       // current position (rad or m)

    // Clamp value to joint limits
    void clamp() {
        value = ::clamp(value, limitLo, limitHi);
    }

    // Local joint transform applied ON TOP OF the fixed body offset
    Mat4 transform() const {
        switch(type) {
            case JointType::Revolute:
                return Mat4::fromQuaternion(
                    Quaternion::fromAxisAngle(axis, value));

            case JointType::Prismatic:
                return Mat4::translation(axis * value);

            case JointType::Fixed:
            default:
                return Mat4::identity();

            // Free / Ball handled externally via a full Transform
        }
    }

    bool hasDOF() const {
        return type != JointType::Fixed;
    }
};
