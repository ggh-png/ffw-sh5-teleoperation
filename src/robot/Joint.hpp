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
    JointType   type     = JointType::Fixed;
    Vec3        axis     = {0, 0, 1};
    bool        hasLimits = false; // false = unlimited (e.g. continuous wheel drive)
    float       limitLo  = -kPi;
    float       limitHi  =  kPi;
    float       value    = 0.f;

    // ── Dynamics (from MJCF <default> joint / <position> actuator) ─────────
    float damping      = 0.f;    // Ns/rad  — viscous joint friction
    float frictionLoss = 0.f;    // Nm      — Coulomb (dry) friction
    float armature     = 0.f;    // kg·m²   — reflected rotor inertia
    float kp           = 0.f;    // Nm/rad  — position servo gain
    float forceMin     = -1e9f;  // Nm      — actuator force range lower bound
    float forceMax     =  1e9f;  // Nm      — actuator force range upper bound
    bool  isVelocityCtrl = false; // true = velocity controller (kp = kv); skip gravity sag

    // ── Runtime physics state (written by JointDynamics, read by transform()) ─
    // valueSag: gravity-induced static deflection [rad or m].
    // Non-destructive — user's joint.value is not modified.
    float valueSag = 0.f;

    void clamp() {
        if(hasLimits) value = ::clamp(value, limitLo, limitHi);
    }

    // Local joint transform applied ON TOP OF the fixed body offset.
    // Uses value + valueSag so gravity sag is transparent to control code.
    Mat4 transform() const {
        const float q = value + valueSag;
        switch(type) {
            case JointType::Revolute:
                return Mat4::fromQuaternion(
                    Quaternion::fromAxisAngle(axis, q));

            case JointType::Prismatic:
                return Mat4::translation(axis * q);

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
