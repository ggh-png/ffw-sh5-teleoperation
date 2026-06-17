#pragma once
#include "Mat4.hpp"

struct Transform {
    Vec3       pos   {0,0,0};
    Quaternion rot   {};          // identity
    Vec3       scale {1,1,1};

    Mat4 toMat4() const {
        return Mat4::trs(pos, rot, scale);
    }

    // Compose: apply this transform after parent
    Transform operator*(const Transform& child) const {
        Transform r;
        r.rot   = (rot * child.rot).normalized();
        r.pos   = pos + rot.rotate(child.pos * scale);
        r.scale = { scale.x * child.scale.x,
                    scale.y * child.scale.y,
                    scale.z * child.scale.z };
        return r;
    }
};
