#pragma once
#include "math/Math.hpp"
#include "robot/SceneNode.hpp"
#include "io/STLLoader.hpp"
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <memory>
#include <vector>
#include <string>
#include <array>

// Physical finger dynamics for both hands (Featherstone btMultiBody).
//
// Each hand = one btMultiBody:
//   base  = hx5_l/r_base (kinematic, follows arm FK)
//   links = finger_l/r_link* (dynamic, PD-servo toward HandPanel targets)
//
// Finger ConvexHull colliders use ColGroup::HAND so they physically interact
// with OBJ (can).  Bullet friction forces hold the can when fingers close —
// no kinematic attachment required.
//
// Frame flow (call in this order):
//   1. HandPanel::applyToModel()   → node->joint.value = target angles
//   2. setPalmTransforms(L, R)     → kinematic bases follow arm FK
//   3. applyGripTorques()          → τ = kp*(target-q) - kd*qdot
//   4. physics.step(dt)
//   5. syncToFK()                  → node->joint.value = physics result
//   6. model->update()             → render fingers at physics positions
class HandPhysics {
public:
    // ── Tunable PD parameters ────────────────────────────────────────────────
    // kp=20: ωn=141 rad/s at I_min=1e-3, dt_sub=0.002 → dt·ωn=0.28 < 2 → stable
    // kd=0.3: ζ≈1.1 (slightly overdamped), settles quickly without oscillation
    // forceMax=5: ~17× the torque needed to hold a 350 g can through one joint
    float kp       = 20.0f;
    float kd       =  0.3f;
    float forceMax =  5.0f;

    void build(btMultiBodyDynamicsWorld* world,
               SceneNode* palmL, SceneNode* palmR,
               const std::vector<std::string>& meshPaths);

    // Update kinematic palm transforms each frame (after FK/IK).
    void setPalmTransforms(const Mat4& palmLWorld, const Mat4& palmRWorld);

    // Apply PD servo torques toward node->joint.value targets.
    void applyGripTorques();

    // Copy Featherstone joint positions back to SceneNode::joint::value.
    void syncToFK();

    void clear(btMultiBodyDynamicsWorld* world);

    bool isBuilt() const { return m_hands[0].mb || m_hands[1].mb; }
    int  linkCount(int side) const {
        return (side>=0&&side<2&&m_hands[side].mb)
               ? m_hands[side].mb->getNumLinks() : 0;
    }

private:
    // Broadphase filter: allow cross-finger collision while blocking
    // within-finger-chain pairs that would cause explosive impulses.
    //
    // Hand link colliders are tagged with userIndex = side*40 + chain*10 + localIdx.
    // Two colliders on the same (side, chain) share userIndex/10 → blocked.
    // All other objects have userIndex < 0 (Bullet default) → always allowed.
    struct FingerChainFilter : btOverlapFilterCallback {
        bool needBroadphaseCollision(btBroadphaseProxy* p0,
                                     btBroadphaseProxy* p1) const override {
            bool defaultOk =
                (p0->m_collisionFilterMask & p1->m_collisionFilterGroup) != 0 &&
                (p1->m_collisionFilterMask & p0->m_collisionFilterGroup) != 0;
            if(!defaultOk) return false;
            auto* o0 = static_cast<btCollisionObject*>(p0->m_clientObject);
            auto* o1 = static_cast<btCollisionObject*>(p1->m_clientObject);
            int u0 = o0 ? o0->getUserIndex() : -1;
            int u1 = o1 ? o1->getUserIndex() : -1;
            if(u0 < 0 || u1 < 0) return true;
            return (u0 / 10) != (u1 / 10);
        }
    };
    FingerChainFilter m_chainFilter;

    struct LinkInfo {
        SceneNode* node;
        int        mbLinkIdx;
        float      kp, damping, forceMin, forceMax;
    };

    struct HandData {
        btMultiBody*                                              mb = nullptr;
        std::vector<LinkInfo>                                     links;
        std::vector<std::unique_ptr<btMultiBodyLinkCollider>>     colliders;
        std::vector<std::unique_ptr<btConvexHullShape>>           shapes;
        std::vector<std::unique_ptr<btMultiBodyJointLimitConstraint>> limits;
    };

    std::array<HandData, 2>   m_hands;
    btMultiBodyDynamicsWorld* m_world = nullptr;

    void buildHand(int side, btMultiBodyDynamicsWorld* world,
                   SceneNode* palmNode,
                   const std::vector<std::string>& meshPaths);

    static bool shouldInclude(const SceneNode* node);
    static btConvexHullShape* buildShape(const SceneNode* node,
                                          const std::vector<std::string>& meshPaths);
};
