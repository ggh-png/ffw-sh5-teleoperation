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
//   ground plane    ↔ everything
//   table / env box ↔ cans, chassis, FINGERS (fingers don't go through table)
//   chassis         ↔ env, cans
//   arm links       ↔ cans  (push during approach)
//   finger links    ↔ cans (grasping), env floor+table (floor stop), no FINGER-FINGER
//   can (free)      ↔ env, chassis, arm links, fingers, OTHER CANS
//   can (dragged)   ↔ env, chassis  (mouse drag: kinematic)
//
// FINGER↔FINGER: intentionally excluded from broadphase — kinematic bodies cannot
//   physically respond to each other.  Thumb-finger penetration is detected via
//   contactPairTest in main.cpp (bypasses broadphase) and fixed at the FK level.

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

    // Table / static env boxes: cans, chassis, AND finger links.
    // Without FINGER here, fingers would pass straight through the table top.
    static constexpr int MASK_TABLE      = OBJ | ROBOT_BASE | FINGER;

    static constexpr int MASK_ROBOT_BASE = ENV | OBJ;

    // Arm links: only interact with graspable objects.
    // Excluding ENV avoids generating arm↔floor manifolds on every frame.
    static constexpr int MASK_ROBOT_LINK = OBJ;

    // Finger links: contact with cans (grasping) and env (floor + table).
    // FINGER↔FINGER is handled manually via contactPairTest in main.cpp.
    static constexpr int MASK_FINGER = OBJ | ENV;

    // Legacy Featherstone mask (unused but kept for compatibility)
    static constexpr int MASK_HAND = ENV | OBJ | HAND;

    // ── Dynamic masks for GraspObj ────────────────────────────────────────────
    // OBJ in free mask → dynamic objects (cans) collide with each other.
    // FINGER in free mask → cans detect and respond to finger bodies.
    static constexpr int MASK_OBJ_FREE = ENV | ROBOT_BASE | ROBOT_LINK | FINGER | OBJ;
    static constexpr int MASK_OBJ_HELD = ENV | ROBOT_BASE;  // mouse drag: kinematic
    static constexpr int MASK_OBJ      = MASK_OBJ_FREE;
}
