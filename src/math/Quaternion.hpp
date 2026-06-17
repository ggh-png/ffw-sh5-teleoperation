#pragma once
#include "Vec3.hpp"
#include <cmath>

// Unit quaternion — MuJoCo convention: w x y z (scalar first)
struct Quaternion {
    float w = 1, x = 0, y = 0, z = 0;

    Quaternion() = default;
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    static Quaternion identity() { return {1,0,0,0}; }

    static Quaternion fromAxisAngle(const Vec3& axis, float angleRad) {
        float s  = std::sin(angleRad * 0.5f);
        Vec3  a  = axis.normalized();
        return { std::cos(angleRad * 0.5f), a.x*s, a.y*s, a.z*s };
    }

    // Euler angles in radians, intrinsic XYZ order (same as MJCF default)
    static Quaternion fromEulerXYZ(float rx, float ry, float rz) {
        auto qx = fromAxisAngle({1,0,0}, rx);
        auto qy = fromAxisAngle({0,1,0}, ry);
        auto qz = fromAxisAngle({0,0,1}, rz);
        return qx * qy * qz;
    }

    Quaternion operator*(const Quaternion& o) const {
        return {
            w*o.w - x*o.x - y*o.y - z*o.z,
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w
        };
    }

    Quaternion conjugate()  const { return {w, -x, -y, -z}; }
    float      normSq()     const { return w*w + x*x + y*y + z*z; }
    float      norm()       const { return std::sqrt(normSq()); }
    Quaternion normalized() const {
        float n = norm();
        return n > 1e-8f ? Quaternion{w/n, x/n, y/n, z/n} : identity();
    }

    // Rotate a vector: v' = q v q*
    Vec3 rotate(const Vec3& v) const {
        // Efficient formula: v' = v + 2w(q×v) + 2(q×(q×v))
        Vec3 qv = {x, y, z};
        Vec3 t  = 2.0f * qv.cross(v);
        return v + w * t + qv.cross(t);
    }

    // Spherical linear interpolation
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, float t) {
        float dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
        Quaternion bb = dot < 0 ? Quaternion{-b.w,-b.x,-b.y,-b.z} : b;
        dot = std::fabs(dot);
        if(dot > 0.9995f) {
            Quaternion r = {a.w+(bb.w-a.w)*t, a.x+(bb.x-a.x)*t,
                            a.y+(bb.y-a.y)*t, a.z+(bb.z-a.z)*t};
            return r.normalized();
        }
        float theta0 = std::acos(dot);
        float theta  = theta0 * t;
        float s0     = std::cos(theta) - dot * std::sin(theta) / std::sin(theta0);
        float s1     = std::sin(theta) / std::sin(theta0);
        return { s0*a.w + s1*bb.w, s0*a.x + s1*bb.x,
                 s0*a.y + s1*bb.y, s0*a.z + s1*bb.z };
    }
};
