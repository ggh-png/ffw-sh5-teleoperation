#pragma once
#include "math/Math.hpp"
#include "io/MJCFParser.hpp"
#include "io/SceneLoader.hpp"
#include "physics/CollisionGroups.hpp"
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <memory>
#include <vector>

// ── GraspObj ──────────────────────────────────────────────────────────────────
// A dynamic rigid body (cola can) that can be grasped by either hand.
//
// Grasping strategy: btFixedConstraint connects the kinematic palm body to
// this (dynamic) body.  The can stays fully dynamic — it has mass, responds
// to gravity, and collides with the table and chassis — only the constraint
// pulls it along with the palm.  This is fundamentally different from the old
// CF_KINEMATIC_OBJECT trick which made the can pass through everything.
struct GraspObj {
    std::unique_ptr<btCollisionShape>     shape;      // btCylinderShape or convex hull
    std::unique_ptr<btDefaultMotionState> state;
    std::unique_ptr<btRigidBody>          body;
    std::unique_ptr<btFixedConstraint>    constraint; // non-null while held
    btTransform initialTransform;                      // spawn pose for resetObjects()
    Vec3  color      = {0.8f, 0.1f, 0.1f};
    float radius     = 0.033f;
    float halfHeight = 0.06f;
    float mass       = 0.35f;
    int   graspedBy  = -1;   // -1 = free, 0 = left hand, 1 = right hand
    bool  mouseDrag  = false;
    bool  isMesh     = false;
};

// ── StaticObj ─────────────────────────────────────────────────────────────────
struct StaticObj {
    std::unique_ptr<btBoxShape>           shape;
    std::unique_ptr<btDefaultMotionState> state;
    std::unique_ptr<btRigidBody>          body;
    Vec3 pos;
    Vec3 halfExtents;
    Vec3 color;
};

// ── PhysicsWorld ──────────────────────────────────────────────────────────────
class PhysicsWorld {
public:
    // Grip fires when palm-centre-to-can-surface distance < kGraspRadius.
    // 8 cm gives a comfortable "hand around can" feel without being a force-field.
    static constexpr float kGraspRadius = 0.08f;

    explicit PhysicsWorld(const PhysicsOptions& opts = {});
    ~PhysicsWorld();

    void step(float dt);

    btMultiBodyDynamicsWorld* bulletWorld() { return m_world.get(); }

    void spawnObjects(const std::vector<ObjectDesc>& descs);

    // ── Mobile base ──────────────────────────────────────────────────────────
    Vec3       basePosition()    const;
    Quaternion baseOrientation() const;
    float      baseYaw()         const { return m_baseYaw; }

    bool setBaseVelocity(const Vec3& robotLocalVel, float yawRate, float dt);
    void setBaseKinematic(bool kinematic);
    bool isGrounded() const;

    // ── Dynamic objects ──────────────────────────────────────────────────────
    int addCylinder(float r, float halfH, float mass,
                    const Vec3& pos, const Vec3& color);
    int addMeshObject(const std::string& stlPath, float scale, float mass,
                      const Vec3& pos, const Vec3& color, float friction = 0.5f);

    // Nearest-object surface distance from palm (for HUD proximity ring).
    float handNearestDist(int side) const;

    // Core grasp interface — call every frame with current palm FK pose.
    //   grip = 0     → release held object (with throw velocity)
    //   grip > 0.05  → try to grasp nearest object within kGraspRadius
    //
    // Internally:
    //   • Moves kinematic palm body to palmPos/palmRot.
    //   • On first frame within range: creates btFixedConstraint(palm, can).
    //     Can stays dynamic — collides with table, follows palm via constraint.
    //   • On release: removes constraint, applies throw velocity.
    void applyGripForce(int side, float grip,
                        const Vec3& palmPos, const Quaternion& palmRot, float dt);

    bool isGrasping(int side) const {
        for(const auto& obj : m_objects) if(obj.graspedBy == side) return true;
        return false;
    }

    // Release all constraints and teleport objects back to spawn positions.
    void resetObjects();

    // ── Mouse drag (RMB) ─────────────────────────────────────────────────────
    int  pickObject(const Vec3& rayOrigin, const Vec3& rayDir) const;
    void beginMouseDrag(int idx);
    void setMouseDragPosition(int idx, const Vec3& pos);
    void endMouseDrag(int idx);

    // ── Object / box state snapshots (for renderer) ──────────────────────────
    struct ObjState {
        Vec3 pos; Quaternion rot;
        Vec3 color; float radius; float halfHeight;
        bool isMesh = false;
    };
    std::vector<ObjState> objectStates() const;

    int addStaticBox(const Vec3& halfExtents, const Vec3& pos, const Vec3& color);
    struct StaticBoxState { Vec3 pos; Vec3 halfExtents; Vec3 color; };
    std::vector<StaticBoxState> staticBoxStates() const;

    btRigidBody* baseBody()          const { return m_base.get(); }
    btRigidBody* staticBoxBody(int i) const {
        return (i >= 0 && i < (int)m_staticBoxes.size())
               ? m_staticBoxes[i].body.get() : nullptr;
    }
    int staticBoxCount() const { return (int)m_staticBoxes.size(); }

private:
    std::unique_ptr<btDefaultCollisionConfiguration> m_colConfig;
    std::unique_ptr<btCollisionDispatcher>           m_dispatcher;
    std::unique_ptr<btBroadphaseInterface>           m_broadphase;
    std::unique_ptr<btMultiBodyConstraintSolver>     m_solver;
    std::unique_ptr<btMultiBodyDynamicsWorld>        m_world;

    // Ground plane
    std::unique_ptr<btStaticPlaneShape>   m_groundShape;
    std::unique_ptr<btDefaultMotionState> m_groundState;
    std::unique_ptr<btRigidBody>          m_ground;

    // Robot chassis (kinematic box, ROBOT_BASE group)
    std::unique_ptr<btBoxShape>           m_baseShape;
    std::unique_ptr<btDefaultMotionState> m_baseState;
    std::unique_ptr<btRigidBody>          m_base;

    // Palm bodies: kinematic spheres used as btFixedConstraint anchors.
    // MASK_PALM = 0 — not in the broadphase, no physical contact with anything.
    // They simply track hx5_l/r_base FK and anchor the grasped object's constraint.
    std::unique_ptr<btSphereShape>          m_palmShape[2];
    std::unique_ptr<btDefaultMotionState>   m_palmState[2];
    std::unique_ptr<btRigidBody>            m_palmBody[2];

    std::vector<GraspObj>  m_objects;
    std::vector<StaticObj> m_staticBoxes;

    Vec3  m_palmPos[2]     = {};
    Vec3  m_palmPosPrev[2] = {};
    float m_baseYaw        = 0.f;
    float m_timestep       = 1.f / 240.f;
    float m_impratio       = 1.f;

    static btRigidBody* makeRigidBody(btCollisionShape*, btMotionState*, float mass);
};
