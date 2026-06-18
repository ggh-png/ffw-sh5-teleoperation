#pragma once
#include "SceneNode.hpp"
#include "math/Math.hpp"
#include <cmath>

// ── MuJoCo-style passive joint dynamics ──────────────────────────────────────
//
// Computes gravity-induced static sag for every position-controlled revolute
// joint, using the inertial data (<inertial>) and servo parameters (kp,
// frictionloss, forcerange) parsed from the MJCF <default> hierarchy.
//
// Physics model (quasi-static, matching MuJoCo's position servo):
//
//   Gravity torque:   τ = Σᵢ (rᵢ × mᵢ·g) · â    (over all descendant links)
//   Coulomb deadband: if |τ| ≤ frictionLoss → no sag
//   Effective torque: τ_eff = τ - frictionLoss · sign(τ)
//   Static deflection:δθ = τ_eff / kp           (position servo equilibrium)
//   Force clamp:      |kp · δθ| ≤ forceMax       (servo saturation)
//
// Writes Joint::valueSag which is transparently added in Joint::transform().
// Calling code pattern (per frame):
//   1. model->update()                           // FK with user-set joint.value
//   2. JointDynamics::apply(*model->root, g)     // writes joint.valueSag
//   3. model->update()                           // FK with sag applied
//
class JointDynamics {
public:
    // Apply gravity sag to all eligible joints.
    // gravity: world-space gravity vector (Y-up, e.g. {0, -9.81, 0}).
    static void apply(SceneNode& root, const Vec3& gravity) {
        visitNode(&root, gravity);
    }

    // Zero all valueSag values (call when disabling dynamics or resetting).
    static void clearSag(SceneNode& root) {
        root.joint.valueSag = 0.f;
        for(auto& c : root.children) clearSag(*c);
    }

private:
    // ── Tree traversal ────────────────────────────────────────────────────

    static void visitNode(SceneNode* node, const Vec3& gravity) {
        if(node->joint.kp > 0.f && !node->joint.isVelocityCtrl && node->parent) {
            if(node->joint.type == JointType::Revolute)
                sagForRevoluteJoint(node, gravity);
            else if(node->joint.type == JointType::Prismatic)
                sagForPrismaticJoint(node, gravity);
        }
        for(auto& c : node->children)
            visitNode(c.get(), gravity);
    }

    // ── Gravity sag for a single revolute joint ───────────────────────────

    static void sagForJoint(SceneNode* node, const Vec3& gravity) {
        sagForRevoluteJoint(node, gravity);
    }

    static void sagForRevoluteJoint(SceneNode* node, const Vec3& gravity) {
        // Joint pivot position and rotation axis in world space.
        // The pivot is the body origin BEFORE the DOF rotation:
        //   T_pivot = parent.worldTransform * node.localTransform()
        // This correctly separates the fixed offset (localTransform) from
        // the variable DOF (joint.transform) so the axis is in parent frame.
        Mat4 Tpivot = node->parent->worldTransform * node->localTransform();
        Vec3 pivotPos  = Tpivot.transformPoint({0.f, 0.f, 0.f});
        Vec3 pivotAxis = Tpivot.transformDir(node->joint.axis).normalized();

        // Accumulate gravity torque from this node and all descendants.
        float tau = 0.f;
        accumTorque(node, pivotPos, pivotAxis, gravity, tau);

        const Joint& j = node->joint;

        // ── Coulomb friction deadband ─────────────────────────────────────
        // If the gravity torque is smaller than static friction, the joint
        // holds position without deflection (MuJoCo frictionloss behavior).
        float absTau = tau < 0.f ? -tau : tau;
        if(absTau <= j.frictionLoss) {
            node->joint.valueSag = 0.f;
            return;
        }
        float sgn = tau > 0.f ? 1.f : -1.f;
        float tauEff = tau - sgn * j.frictionLoss;

        // ── Static equilibrium deflection ─────────────────────────────────
        // Position servo: τ_servo = kp · (q_des - q_actual)
        // At equilibrium:  kp · δθ = τ_gravity  →  δθ = τ / kp
        // Armature adds inertia but does NOT change static equilibrium —
        // only the dynamic transient response is affected.
        float sag = tauEff / j.kp;

        // ── Force/torque saturation ───────────────────────────────────────
        // The servo is limited to [forceMin, forceMax] (N·m).
        // When gravity exceeds forceMax the servo saturates and the joint
        // yields to its limit — cap sag at the force-limited deflection.
        float sagMax = j.forceMax / j.kp;  // forceMax > 0
        float sagMin = j.forceMin / j.kp;  // forceMin < 0
        if(sag >  sagMax) sag =  sagMax;
        if(sag <  sagMin) sag =  sagMin;

        node->joint.valueSag = sag;
    }

    // ── Gravity sag for a single prismatic (slide) joint ─────────────────
    //
    // For a prismatic joint the relevant quantity is the gravity FORCE along
    // the slide axis (not torque):
    //   F = Σᵢ (mᵢ · g) · â       (dot product onto joint axis)
    //   sag = (F - sign(F)·frictionLoss) / kp   [meters]
    // Saturated at [forceMin, forceMax] / kp.

    static void sagForPrismaticJoint(SceneNode* node, const Vec3& gravity) {
        // Axis in world space
        Mat4 Tpivot   = node->parent->worldTransform * node->localTransform();
        Vec3 axisWorld = Tpivot.transformDir(node->joint.axis).normalized();

        // Accumulate force component along the slide axis from all descendants
        float F = 0.f;
        accumForce(node, axisWorld, gravity, F);

        const Joint& j = node->joint;
        float absF = F < 0.f ? -F : F;
        if(absF <= j.frictionLoss) {
            node->joint.valueSag = 0.f;
            return;
        }
        float sgn    = F > 0.f ? 1.f : -1.f;
        float Feff   = F - sgn * j.frictionLoss;
        float sag    = Feff / j.kp;
        float sagMax = j.forceMax / j.kp;
        float sagMin = j.forceMin / j.kp;
        if(sag >  sagMax) sag =  sagMax;
        if(sag <  sagMin) sag =  sagMin;
        node->joint.valueSag = sag;
    }

    // ── Gravity force accumulator for prismatic joints ────────────────────

    static void accumForce(const SceneNode* node,
                            const Vec3&      axis,
                            const Vec3&      gravity,
                            float&           F)
    {
        if(node->inertial.valid && node->inertial.mass > 0.f) {
            Vec3 gF = gravity * node->inertial.mass;
            F += gF.dot(axis);  // force component along slide axis
        }
        for(const auto& child : node->children)
            accumForce(child.get(), axis, gravity, F);
    }

    // ── Gravity torque accumulator (DFS over descendants) ─────────────────
    //
    // For each link i:  τᵢ = (rᵢ × mᵢ·g) · â
    //   rᵢ = com_world_i − pivotPos   (lever arm)
    //   g   = gravity vector
    //   â   = joint axis (unit vector in world space)
    //
    // inertial.com is in the body's MJCF-local frame.
    // worldTransform maps body-local → world, so:
    //   com_world = worldTransform.transformPoint(inertial.com)
    // is correct because worldTransform already includes the root −90° X
    // rotation that converts MJCF Z-up to Y-up world space.

    static void accumTorque(const SceneNode* node,
                             const Vec3&      pivotPos,
                             const Vec3&      pivotAxis,
                             const Vec3&      gravity,
                             float&           tau)
    {
        if(node->inertial.valid && node->inertial.mass > 0.f) {
            Vec3 comW = node->worldTransform.transformPoint(node->inertial.com);
            Vec3 r    = comW - pivotPos;
            Vec3 gF   = gravity * node->inertial.mass;   // gravity force on link
            // Torque = r × F, projected onto joint axis
            tau += r.cross(gF).dot(pivotAxis);
        }
        for(const auto& child : node->children)
            accumTorque(child.get(), pivotPos, pivotAxis, gravity, tau);
    }
};
