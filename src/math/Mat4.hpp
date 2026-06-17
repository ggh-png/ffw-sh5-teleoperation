#pragma once
#include "Vec3.hpp"
#include "Vec4.hpp"
#include "Quaternion.hpp"
#include <cmath>
#include <cstring>

// Column-major 4x4 matrix (OpenGL convention)
// data[col*4 + row]  →  column c, row r
struct Mat4 {
    float data[16];

    Mat4() {
        std::memset(data, 0, sizeof(data));
        data[0] = data[5] = data[10] = data[15] = 1.f; // identity
    }

    static Mat4 identity() { return Mat4{}; }

    static Mat4 zero() {
        Mat4 m;
        std::memset(m.data, 0, sizeof(m.data));
        return m;
    }

    float&       at(int row, int col)       { return data[col*4 + row]; }
    float        at(int row, int col) const { return data[col*4 + row]; }
    const float* ptr()                const { return data; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    Mat4 operator*(const Mat4& o) const {
        Mat4 r = zero();
        for(int c = 0; c < 4; ++c)
            for(int rr = 0; rr < 4; ++rr)
                for(int k = 0; k < 4; ++k)
                    r.at(rr,c) += at(rr,k) * o.at(k,c);
        return r;
    }

    Vec4 operator*(const Vec4& v) const {
        return {
            at(0,0)*v.x + at(0,1)*v.y + at(0,2)*v.z + at(0,3)*v.w,
            at(1,0)*v.x + at(1,1)*v.y + at(1,2)*v.z + at(1,3)*v.w,
            at(2,0)*v.x + at(2,1)*v.y + at(2,2)*v.z + at(2,3)*v.w,
            at(3,0)*v.x + at(3,1)*v.y + at(3,2)*v.z + at(3,3)*v.w
        };
    }

    Vec3 transformPoint(const Vec3& p) const {
        Vec4 r = *this * Vec4(p, 1.f);
        return r.xyz() / r.w;
    }

    Vec3 transformDir(const Vec3& d) const {
        Vec4 r = *this * Vec4(d, 0.f);
        return r.xyz();
    }

    Mat4 transposed() const {
        Mat4 r = zero();
        for(int c = 0; c < 4; ++c)
            for(int rr = 0; rr < 4; ++rr)
                r.at(c, rr) = at(rr, c);
        return r;
    }

    // ── Factory methods ──────────────────────────────────────────────────────
    static Mat4 translation(const Vec3& t) {
        Mat4 m;
        m.at(0,3) = t.x;
        m.at(1,3) = t.y;
        m.at(2,3) = t.z;
        return m;
    }

    static Mat4 scaling(const Vec3& s) {
        Mat4 m = zero();
        m.at(0,0) = s.x;
        m.at(1,1) = s.y;
        m.at(2,2) = s.z;
        m.at(3,3) = 1.f;
        return m;
    }

    static Mat4 fromQuaternion(const Quaternion& q) {
        float w=q.w, x=q.x, y=q.y, z=q.z;
        Mat4 m = zero();
        m.at(0,0) = 1 - 2*(y*y + z*z);
        m.at(0,1) =     2*(x*y - w*z);
        m.at(0,2) =     2*(x*z + w*y);
        m.at(1,0) =     2*(x*y + w*z);
        m.at(1,1) = 1 - 2*(x*x + z*z);
        m.at(1,2) =     2*(y*z - w*x);
        m.at(2,0) =     2*(x*z - w*y);
        m.at(2,1) =     2*(y*z + w*x);
        m.at(2,2) = 1 - 2*(x*x + y*y);
        m.at(3,3) = 1.f;
        return m;
    }

    // TRS: translation * rotation * scale
    static Mat4 trs(const Vec3& t, const Quaternion& r, const Vec3& s) {
        return translation(t) * fromQuaternion(r) * scaling(s);
    }

    // View matrix (right-handed, -Z forward)
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = (center - eye).normalized();
        Vec3 r = f.cross(up).normalized();
        Vec3 u = r.cross(f);

        Mat4 m = zero();
        m.at(0,0) =  r.x;  m.at(0,1) =  r.y;  m.at(0,2) =  r.z;  m.at(0,3) = -r.dot(eye);
        m.at(1,0) =  u.x;  m.at(1,1) =  u.y;  m.at(1,2) =  u.z;  m.at(1,3) = -u.dot(eye);
        m.at(2,0) = -f.x;  m.at(2,1) = -f.y;  m.at(2,2) = -f.z;  m.at(2,3) =  f.dot(eye);
        m.at(3,3) = 1.f;
        return m;
    }

    // Perspective projection (vertical fov in radians)
    static Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
        float t = std::tan(fovY * 0.5f);
        Mat4 m = zero();
        m.at(0,0) = 1.f / (aspect * t);
        m.at(1,1) = 1.f / t;
        m.at(2,2) = -(zFar + zNear) / (zFar - zNear);
        m.at(2,3) = -(2.f * zFar * zNear) / (zFar - zNear);
        m.at(3,2) = -1.f;
        return m;
    }

    // Normal matrix (upper-left 3×3 of inverse-transpose)
    // Approximation valid when uniform scale: just return upper-left
    Mat4 normalMatrix() const {
        Mat4 n = zero();
        n.at(0,0) = at(0,0); n.at(0,1) = at(0,1); n.at(0,2) = at(0,2);
        n.at(1,0) = at(1,0); n.at(1,1) = at(1,1); n.at(1,2) = at(1,2);
        n.at(2,0) = at(2,0); n.at(2,1) = at(2,1); n.at(2,2) = at(2,2);
        n.at(3,3) = 1.f;
        return n.transposed(); // inverse = transposed for rotation-only
    }
};
