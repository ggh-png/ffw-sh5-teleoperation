#pragma once

// Bullet collision filter groups.
// Each rigid body belongs to one group and has a mask of groups it accepts hits from.
// Bullet's broadphase respects these flags; contactPairTest (used by Jacobian correction
// in RobotCollider::findStaticPenetrations) intentionally bypasses them.
//
// Bit layout:
//   ENV        (0x01)  ground plane + static boxes (table)
//   ROBOT_BASE (0x02)  kinematic base box that moves with the robot
//   ROBOT_LINK (0x04)  arm-link convex hulls — physics-invisible, contact-tested manually
//   OBJ        (0x08)  graspable dynamic objects (cola can, etc.)
//   HAND       (0x10)  palm + fingertip kinematic spheres
//
// Desired interaction matrix ("→" = A generates Bullet contact with B):
//   ground    → everything
//   table     → OBJ, HAND, ROBOT_BASE
//   base      → ENV, OBJ
//   arm links → (none via Bullet broadphase — Bullet-invisible)
//   can       → ENV, HAND, ROBOT_BASE
//   hand      → ENV, OBJ, HAND
//
// Why MASK_ROBOT_LINK = 0 (Bullet-invisible):
//   Arm links are kinematic (infinite effective mass). Any overlap with a dynamic
//   body (can) generates an enormous impulse that sends the can flying, even if
//   the arm is not visually near the can — ConvexHull margin and initial pose
//   can cause immediate penetration at spawn.
//
//   Grasping is now handled entirely by HandPhysics finger colliders (HAND group).
//   Arm-env penetration is cosmetic; IK workspace clamping limits most cases.

namespace ColGroup {
    static constexpr int ENV        = 1 << 0;
    static constexpr int ROBOT_BASE = 1 << 1;
    static constexpr int ROBOT_LINK = 1 << 2;   // registered but Bullet-invisible (mask=0)
    static constexpr int OBJ        = 1 << 3;
    static constexpr int HAND       = 1 << 4;

    static constexpr int ALL        = -1;

    // Convenience masks
    static constexpr int MASK_ENV        = ALL;
    static constexpr int MASK_TABLE      = OBJ | HAND | ROBOT_BASE;
    static constexpr int MASK_ROBOT_BASE = ENV | OBJ;
    static constexpr int MASK_ROBOT_LINK = 0;      // Bullet-invisible (see above)
    static constexpr int MASK_OBJ        = ENV | HAND | ROBOT_BASE;
    static constexpr int MASK_HAND       = ENV | OBJ | HAND;
}
