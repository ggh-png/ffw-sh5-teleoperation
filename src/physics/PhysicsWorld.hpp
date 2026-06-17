#pragma once
#include "math/Math.hpp"
#include <btBulletDynamicsCommon.h>
#include <memory>
#include <vector>

// Thin Bullet3 wrapper.
// Manages: ground plane + robot base rigid body.
// All articulated joints are handled by FK — Bullet only simulates
// the mobile base (gravity, floor contact, velocity drive).
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    // Step the simulation (call once per frame)
    void step(float dt);

    // Mobile base: kinematic body driven by teleoperation input
    // Returns the base's world position (to feed back into FK root)
    Vec3 basePosition()    const;
    void setBaseVelocity(const Vec3& linearVel); // m/s in world frame

    // Toggle between kinematic (teleop) and dynamic (free fall) base
    void setBaseKinematic(bool kinematic);

private:
    std::unique_ptr<btDefaultCollisionConfiguration>     m_colConfig;
    std::unique_ptr<btCollisionDispatcher>               m_dispatcher;
    std::unique_ptr<btBroadphaseInterface>               m_broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDiscreteDynamicsWorld>             m_world;

    // Shapes (owned here, rigid bodies reference them)
    std::unique_ptr<btStaticPlaneShape>  m_groundShape;
    std::unique_ptr<btBoxShape>          m_baseShape;

    // Rigid bodies
    std::unique_ptr<btRigidBody>         m_ground;
    std::unique_ptr<btRigidBody>         m_base;

    // Motion states
    std::unique_ptr<btDefaultMotionState> m_groundState;
    std::unique_ptr<btDefaultMotionState> m_baseState;

    static btRigidBody* makeRigidBody(btCollisionShape* shape,
                                      btMotionState*    state,
                                      float             mass);
};
