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
//   ground     ↔ everything
//   table      ↔ can, chassis
//   chassis    ↔ ground/table, can
//   arm+palm   ↔ FREE can only  (mask=OBJ — excluded when can is held by constraint)
//   FREE can   ↔ env, chassis, arm+palm
//   HELD can   ↔ env, chassis   (arm disabled while held to avoid constraint fight)
//   palm sph   → (mask=0, constraint anchor only)
//
// Dynamic masking of GraspObj:
//   MASK_OBJ_FREE  default: can collides with arm links + env + chassis
//   MASK_OBJ_HELD  while btFixedConstraint is active:
//                    can only collides with env + chassis
//                    (prevents kinematic arm from fighting the constraint)
//   applyGripForce / beginMouseDrag / resetObjects call refilterObj() to switch.

namespace ColGroup {
    static constexpr int ENV        = 1 << 0;
    static constexpr int ROBOT_BASE = 1 << 1;
    static constexpr int ROBOT_LINK = 1 << 2;   // arm + palm mesh hulls
    static constexpr int OBJ        = 1 << 3;
    static constexpr int HAND       = 1 << 4;   // HandPhysics legacy
    static constexpr int PALM       = 1 << 5;   // btFixedConstraint anchor (mask=0)

    static constexpr int ALL = -1;

    // Static masks (set at body creation)
    static constexpr int MASK_ENV        = ALL;
    static constexpr int MASK_TABLE      = OBJ | ROBOT_BASE;
    static constexpr int MASK_ROBOT_BASE = ENV | OBJ;
    static constexpr int MASK_ROBOT_LINK = OBJ;           // arm+palm collide with free can
    static constexpr int MASK_HAND       = ENV | OBJ | HAND;
    static constexpr int MASK_PALM       = 0;             // constraint anchor only

    // Dynamic masks for GraspObj — switched via refilterObj()
    static constexpr int MASK_OBJ_FREE   = ENV | ROBOT_BASE | ROBOT_LINK; // default
    static constexpr int MASK_OBJ_HELD   = ENV | ROBOT_BASE;              // held by constraint
    static constexpr int MASK_OBJ        = MASK_OBJ_FREE;                 // alias
}
