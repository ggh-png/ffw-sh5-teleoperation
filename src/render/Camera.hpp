#pragma once
#include "math/Math.hpp"

// Orbit camera: rotates around a target point.
// Mouse left-drag → azimuth/elevation, scroll → zoom, middle-drag → pan.
class Camera {
public:
    Vec3  target   = {0, 0, 0.8f};  // look-at center (robot waist height)
    float radius   = 3.f;           // distance from target
    float azimuth  = toRad(-45.f);  // horizontal angle (rad)
    float elevation= toRad(20.f);   // vertical angle from XZ plane (rad)
    float fovY     = toRad(50.f);
    float zNear    = 0.01f;
    float zFar     = 200.f;

    Vec3 eye() const {
        return target + Vec3{
            radius * std::cos(elevation) * std::sin(azimuth),
            radius * std::sin(elevation),
            radius * std::cos(elevation) * std::cos(azimuth)
        };
    }

    Mat4 view()       const { return Mat4::lookAt(eye(), target, {0,1,0}); }
    Mat4 projection(float aspect) const {
        return Mat4::perspective(fovY, aspect, zNear, zFar);
    }

    // Input handlers (called from InputManager)
    void onMouseDrag(float dx, float dy) {
        azimuth   -= dx * 0.005f;
        elevation  = clamp(elevation + dy * 0.005f,
                           toRad(-89.f), toRad(89.f));
    }

    void onScroll(float dy) {
        radius = clamp(radius - dy * 0.2f, 0.2f, 50.f);
    }

    void onPan(float dx, float dy) {
        // Move target in camera's local XY plane
        Vec3 e = eye();
        Vec3 forward = (target - e).normalized();
        Vec3 right   = forward.cross({0,1,0}).normalized();
        Vec3 up      = right.cross(forward);
        target -= (right * dx - up * dy) * 0.005f * radius;
    }
};
