#pragma once

// Bullet collision filter groups.
// Broadphase contact: generated only when BOTH sides pass —
//   (A.group & B.mask) != 0  &&  (B.group & A.mask) != 0
//
// Groups:
//   ENV        (0x01)  ground plane + static env boxes (table, walls)
//   ROBOT_BASE (0x02)  kinematic chassis box
//   ROBOT_LINK (0x04)  arm link convex hulls (kinematic, NOT fingers)
//   OBJ        (0x08)  graspable dynamic objects (cola can, box, …)
//   HAND       (0x10)  legacy Featherstone finger bodies (unused)
//   FINGER     (0x20)  kinematic finger convex hulls (separate from arm links)
//
// Interaction matrix:
//   ground        ↔ everything
//   table         ↔ can, chassis
//   chassis       ↔ env, can
//   arm links     ↔ can  (push can during approach; don't self-collide with arm)
//   finger links  ↔ can, env (floor/table), other finger links (for penetration detection)
//   can (free)    ↔ env, chassis, arm links, finger links, OTHER CANS
//   can (dragged) ↔ env, chassis  (mouse drag: kinematic, no robot contact)
//
// NOTE — kinematic fingers cannot be physically stopped by other kinematic fingers.
//   FINGER↔FINGER contact manifolds are used only for grip-angle back-off in main.cpp.

namespace ColGroup {
    static constexpr int ENV        = 1 << 0;
    static constexpr int ROBOT_BASE = 1 << 1;
    static constexpr int ROBOT_LINK = 1 << 2;   // arm links (wrist/elbow/shoulder)
    static constexpr int OBJ        = 1 << 3;   // dynamic objects
    static constexpr int HAND       = 1 << 4;   // legacy Featherstone (unused)
    static constexpr int FINGER     = 1 << 5;   // finger convex hulls (kinematic)

    static constexpr int ALL = -1;

    // ── Static masks (set once at body creation) ─────────────────────────────
    static constexpr int MASK_ENV        = ALL;
    static constexpr int MASK_TABLE      = OBJ | ROBOT_BASE;
    static constexpr int MASK_ROBOT_BASE = ENV | OBJ;

    // Arm links: only interact with graspable objects.
    // Do NOT include ENV (would generate huge arm↔floor contact list every frame).
    static constexpr int MASK_ROBOT_LINK = OBJ;

    // Finger links: interact with objects (for grasping force), environment
    // (fingers don't go through floor/table), and other finger links
    // (for contactPairTest-based penetration detection in main.cpp).
    static constexpr int MASK_FINGER = OBJ | ENV | FINGER;

    // Legacy Featherstone mask (unused but kept for compatibility)
    static constexpr int MASK_HAND = ENV | OBJ | HAND;

    // ── Dynamic masks for GraspObj ────────────────────────────────────────────
    // OBJ included in free mask → dynamic objects collide with each other.
    static constexpr int MASK_OBJ_FREE = ENV | ROBOT_BASE | ROBOT_LINK | FINGER | OBJ;
    static constexpr int MASK_OBJ_HELD = ENV | ROBOT_BASE;  // mouse drag: kinematic
    static constexpr int MASK_OBJ      = MASK_OBJ_FREE;
}
