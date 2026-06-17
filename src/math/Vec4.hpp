#pragma once
#include "Vec3.hpp"

struct Vec4 {
    float x = 0, y = 0, z = 0, w = 0;

    Vec4() = default;
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(const Vec3& v, float w)             : x(v.x), y(v.y), z(v.z), w(w) {}

    Vec3  xyz()  const { return {x, y, z}; }
    Vec3  rgb()  const { return {x, y, z}; }

    Vec4 operator+(const Vec4& o) const { return {x+o.x, y+o.y, z+o.z, w+o.w}; }
    Vec4 operator*(float s)       const { return {x*s, y*s, z*s, w*s}; }

    float&       operator[](int i)       { return (&x)[i]; }
    float        operator[](int i) const { return (&x)[i]; }
    const float* ptr()             const { return &x; }
};
