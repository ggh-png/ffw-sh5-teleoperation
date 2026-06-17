#pragma once

#include "Vec3.hpp"
#include "Vec4.hpp"
#include "Quaternion.hpp"
#include "Mat4.hpp"
#include "Transform.hpp"

#include <cmath>
#include <algorithm>

static constexpr float kPi  = 3.14159265358979323846f;
static constexpr float k2Pi = 6.28318530717958647692f;

inline float toRad(float deg) { return deg * kPi / 180.f; }
inline float toDeg(float rad) { return rad * 180.f / kPi; }

inline float clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}
