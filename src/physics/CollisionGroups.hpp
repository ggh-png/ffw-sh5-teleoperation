#pragma once

// Bullet collision filter groups.
// Broadphase contact: generated only when BOTH sides pass —
//   (A.group & B.mask) && (B.group & A.mask)
//
// Groups:
//   ENV        (0x01)  ground plane + static boxes (table)
//   ROBOT_BASE (0x02)  kinematic chassis box
//   ROBOT_LINK (0x04)  arm link + palm convex hulls (kinematic, collide with free can)
//   OBJ        (0x08)  graspable dynamic objects (cola can)
//   HAND       (0x10)  legacy: finger btMultiBody (HandPhysics, not built by default)
//   PALM       (0x20)  kinematic palm spheres — btFixedConstraint anchor (mask=0)
//
// Interaction matrix (desired):
//   ground       ↔ everything
//   table        ↔ can, chassis
//   chassis      ↔ ground/table, can
//   arm+palm     ↔ can (provides normal force for grip)
//   fingers      ↔ can (Featherstone dynamics, friction holds can)
//   can (free)   ↔ env, chassis, arm+palm, fingers
//   can (dragged)↔ env, chassis  (mouse drag: kinematic, no robot contact)
//
// Grasping: purely physical — Featherstone finger colliders close on can,
//   friction forces hold it.  No btFixedConstraint, no rule-based trigger.
//   MASK_OBJ_HELD is used only during RMB mouse drag (can is kinematic).

namespace ColGroup {
    static constexpr int ENV        = 1 << 0;
    static constexpr int ROBOT_BASE = 1 << 1;
    static constexpr int ROBOT_LINK = 1 << 2;   // arm + palm mesh hulls (kinematic)
    static constexpr int OBJ        = 1 << 3;
    static constexpr int HAND       = 1 << 4;   // Featherstone finger link colliders

    static constexpr int ALL = -1;

    // Static masks (set at body creation)
    // ROBOT_LINK kinematic bodies must NOT collide with HAND Featherstone bodies:
    // kinematic bodies apply infinite constraint force → would explode finger dynamics.
    static constexpr int MASK_ENV        = ALL;
    static constexpr int MASK_TABLE      = OBJ | ROBOT_BASE;
    static constexpr int MASK_ROBOT_BASE = ENV | OBJ;
    static constexpr int MASK_ROBOT_LINK = OBJ;                // arm+palm touch can only
    static constexpr int MASK_HAND       = ENV | OBJ | HAND;   // fingers: env, can, cross-finger

    // Dynamic masks for GraspObj — switched via refilterObj()
    static constexpr int MASK_OBJ_FREE = ENV | ROBOT_BASE | ROBOT_LINK | HAND; // normal: all robot parts
    static constexpr int MASK_OBJ_HELD = ENV | ROBOT_BASE;                     // mouse drag kinematic
    static constexpr int MASK_OBJ      = MASK_OBJ_FREE;
}
