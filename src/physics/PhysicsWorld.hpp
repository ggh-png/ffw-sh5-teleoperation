#pragma once
#include "math/Math.hpp"
#include "io/MJCFParser.hpp"
#include "io/SceneLoader.hpp"
#include "physics/CollisionGroups.hpp"
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <memory>
#include <vector>

// Dynamic grabbable object (cola can)
struct GraspObj {
    std::unique_ptr<btCollisionShape>     shape;   // btCylinderShape or convex hull
    std::unique_ptr<btDefaultMotionState> state;
    std::unique_ptr<btRigidBody>          body;
    btTransform initialTransform;      // spawn pose — used by resetObjects()
    btTransform graspRelTransform;     // can pose relative to palm at grasp time
    Vec3  color      = {0.8f, 0.1f, 0.1f};
    float radius     = 0.033f;
    float halfHeight = 0.06f;
    float mass       = 0.35f;
    int   graspedBy  = -1;   // side holding it (-1 = free)
    bool  mouseDrag  = false;
    bool  isMesh     = false; // true = STL mesh visual
};

// Static environment box (table, shelf, ...)
struct StaticObj {
    std::unique_ptr<btBoxShape>           shape;
    std::unique_ptr<btDefaultMotionState> state;
    std::unique_ptr<btRigidBody>          body;
    Vec3 pos;
    Vec3 halfExtents;
    Vec3 color;
};

// ── PhysicsWorld ──────────────────────────────────────────────────────────────
// World based on btMultiBodyDynamicsWorld (Featherstone solver) so that
// ArticulatedRobot btMultiBody links participate in the same contact resolution
// as the mobile base (btRigidBody) and the can (btRigidBody).
class PhysicsWorld {
public:
    static constexpr float kGraspRadius = 0.10f;

    explicit PhysicsWorld(const PhysicsOptions& opts = {});
    ~PhysicsWorld();

    void step(float dt);

    // Expose the underlying world (btMultiBodyDynamicsWorld ⊃ btDiscreteDynamicsWorld).
    // RobotCollider and HandPhysics both use this.
    btMultiBodyDynamicsWorld* bulletWorld() { return m_world.get(); }

    // Spawn objects described by a SceneLoader result.
    // Static boxes go into renderer.boxes; dynamic cylinders into renderer.objects.
    // Returns the index of the first spawned cylinder (for runtime reference).
    void spawnObjects(const std::vector<ObjectDesc>& descs);

    // ── Mobile base ──────────────────────────────────────────────────────────
    Vec3       basePosition()    const;
    Quaternion baseOrientation() const;
    float      baseYaw()         const { return m_baseYaw; }

    bool setBaseVelocity(const Vec3& robotLocalVel, float yawRate, float dt);
    void setBaseKinematic(bool kinematic);
    bool isGrounded() const;

    // ── Dynamic objects (can) ────────────────────────────────────────────────
    int addCylinder(float r, float halfH, float mass,
                    const Vec3& pos, const Vec3& color);

    // STL mesh object: physics = btCylinderShape fitted from mesh bounds,
    // visual = STL mesh rendered by Renderer::m_canMesh.
    int addMeshObject(const std::string& stlPath, float scale, float mass,
                      const Vec3& pos, const Vec3& color, float friction = 0.5f);

    // Proximity check using palm position (from FK / ArticulatedRobot).
    // Returns distance from nearest object surface, for HUD ring.
    float handNearestDist(int side) const;

    // Grasping: call every frame after syncToFK().
    //   palmPos / palmRot: world position and orientation of the palm link (hx5_l/r_base).
    //   gripStrength: 0 = open, >0 = trying to grasp.
    //   dt: frame delta for throw-velocity computation on release.
    void applyGripForce(int side, float gripStrength,
                        const Vec3& palmPos, const Quaternion& palmRot, float dt);

    // Move held objects rigidly with the palm each frame.
    // palmPos / palmRot: current palm world pose (same values as applyGripForce).
    void updateGraspedObjects(int side, const Vec3& palmPos, const Quaternion& palmRot);

    bool isGrasping(int side) const {
        for(const auto& obj : m_objects) if(obj.graspedBy == side) return true;
        return false;
    }

    // Mouse drag (RMB)
    void beginMouseDrag(int idx);
    void setMouseDragPosition(int idx, const Vec3& pos);
    void endMouseDrag(int idx);

    struct ObjState {
        Vec3 pos; Quaternion rot;
        Vec3 color; float radius; float halfHeight;
        bool isMesh = false;
    };
    std::vector<ObjState> objectStates() const;

    // ── Static boxes ─────────────────────────────────────────────────────────
    int addStaticBox(const Vec3& halfExtents, const Vec3& pos, const Vec3& color);

    struct StaticBoxState { Vec3 pos; Vec3 halfExtents; Vec3 color; };
    std::vector<StaticBoxState> staticBoxStates() const;

    btRigidBody* baseBody()         const { return m_base.get(); }
    btRigidBody* staticBoxBody(int i) const {
        return (i>=0&&i<(int)m_staticBoxes.size()) ? m_staticBoxes[i].body.get() : nullptr;
    }
    int staticBoxCount() const { return (int)m_staticBoxes.size(); }

    int pickObject(const Vec3& rayOrigin, const Vec3& rayDir) const;

    // Reset all dynamic objects to their spawn transforms, release all grasps.
    // Call when user presses R (or any "reset scene" action).
    void resetObjects();

    // Update palm positions used by handNearestDist() without triggering grip logic.
    // Must be called every frame when HandPhysics is active (applyGripForce is not called then).
    void updatePalmPositions(const Vec3& palmL, const Vec3& palmR);

private:
    std::unique_ptr<btDefaultCollisionConfiguration> m_colConfig;
    std::unique_ptr<btCollisionDispatcher>           m_dispatcher;
    std::unique_ptr<btBroadphaseInterface>           m_broadphase;
    std::unique_ptr<btMultiBodyConstraintSolver>     m_solver;
    std::unique_ptr<btMultiBodyDynamicsWorld>        m_world;

    std::unique_ptr<btStaticPlaneShape>   m_groundShape;
    std::unique_ptr<btBoxShape>           m_baseShape;
    std::unique_ptr<btRigidBody>          m_ground;
    std::unique_ptr<btRigidBody>          m_base;
    std::unique_ptr<btDefaultMotionState> m_groundState;
    std::unique_ptr<btDefaultMotionState> m_baseState;

    std::vector<GraspObj>  m_objects;
    std::vector<StaticObj> m_staticBoxes;

    // Palm positions tracked per side for throw-velocity on release
    Vec3 m_palmPos[2]     = {};
    Vec3 m_palmPosPrev[2] = {};

    float m_baseYaw  = 0.f;
    float m_timestep = 1.f / 240.f;
    float m_impratio = 1.f;

    static btRigidBody* makeRigidBody(btCollisionShape*, btMotionState*, float mass);
};
