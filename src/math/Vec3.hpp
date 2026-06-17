#pragma once
#include <cmath>

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s)       const { return {x*s, y*s, z*s}; }
    Vec3 operator/(float s)       const { return {x/s, y/s, z/s}; }
    Vec3 operator-()              const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3& operator*=(float s)       { x*=s; y*=s; z*=s; return *this; }

    float  dot(const Vec3& o)   const { return x*o.x + y*o.y + z*o.z; }
    Vec3   cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }

    float  lengthSq() const { return x*x + y*y + z*z; }
    float  length()   const { return std::sqrt(lengthSq()); }
    Vec3   normalized() const {
        float l = length();
        return l > 1e-8f ? *this / l : Vec3{0,0,0};
    }

    float&       operator[](int i)       { return (&x)[i]; }
    float        operator[](int i) const { return (&x)[i]; }
    const float* ptr()             const { return &x; }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline Vec3 mix(const Vec3& a, const Vec3& b, float t) {
    return a * (1.f - t) + b * t;
}
