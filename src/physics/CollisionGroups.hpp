#pragma once

// Bullet collision filter groups.
// Each rigid body belongs to one group and has a mask of groups it can interact with.
// Bullet's broadphase requires BOTH sides to pass: (A.mask & B.group) && (B.mask & A.group).
//
// Bit layout:
//   ENV        (0x01)  ground plane + static boxes (table)
//   ROBOT_BASE (0x02)  kinematic chassis box
//   ROBOT_LINK (0x04)  arm-link convex hulls
//   OBJ        (0x08)  graspable dynamic objects (cola can, etc.)
//   HAND       (0x10)  finger btMultiBody colliders (HandPhysics, legacy)
//   PALM       (0x20)  kinematic palm spheres — btFixedConstraint anchors only
//
// Interaction matrix:
//   ground     ↔ everything
//   table      ↔ can, chassis
//   chassis    ↔ ground/table, can
//   arm links  → (Bullet-invisible: MASK_ROBOT_LINK=0)
//   can        ↔ ground/table, chassis
//   finger     ↔ ground/table, can    (HandPhysics legacy, not built by default)
//   palm       → (constraint-anchor only: MASK_PALM=0, no broadphase interaction)
//
// Why MASK_ROBOT_LINK = 0:
//   Arm links are kinematic (infinite effective mass). Any Bullet contact with a
//   dynamic body (can) generates an enormous impulse that launches the can.
//   Non-penetration during the approach phase is the operator's responsibility.
//
// Why MASK_PALM = 0:
//   Palm spheres are used solely as btFixedConstraint anchor bodies.
//   They must not participate in broadphase collision — they follow FK exactly,
//   and kinematic bodies that touch a dynamic body apply infinite impulse.
//   The constraint itself is what holds the can (not friction / contact).

namespace ColGroup {
    static constexpr int ENV        = 1 << 0;
    static constexpr int ROBOT_BASE = 1 << 1;
    static constexpr int ROBOT_LINK = 1 << 2;   // registered but Bullet-invisible (mask=0)
    static constexpr int OBJ        = 1 << 3;
    static constexpr int HAND       = 1 << 4;   // HandPhysics legacy
    static constexpr int PALM       = 1 << 5;   // btFixedConstraint anchor (mask=0)

    static constexpr int ALL = -1;

    static constexpr int MASK_ENV        = ALL;
    static constexpr int MASK_TABLE      = OBJ | ROBOT_BASE;
    static constexpr int MASK_ROBOT_BASE = ENV | OBJ;
    static constexpr int MASK_ROBOT_LINK = 0;           // Bullet-invisible
    static constexpr int MASK_OBJ        = ENV | ROBOT_BASE;
    static constexpr int MASK_HAND       = ENV | OBJ | HAND;
    static constexpr int MASK_PALM       = 0;           // constraint anchor only
}
