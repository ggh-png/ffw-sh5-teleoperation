#include "PhysicsWorld.hpp"

PhysicsWorld::PhysicsWorld() {
    m_colConfig  = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_colConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver     = std::make_unique<btSequentialImpulseConstraintSolver>();

    m_world = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(),
        m_solver.get(), m_colConfig.get());
    m_world->setGravity({0, -9.81f, 0});

    // ── Ground plane (y=0) ────────────────────────────────────────────────
    m_groundShape = std::make_unique<btStaticPlaneShape>(
        btVector3(0,1,0), 0.f);
    m_groundState = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), {0,0,0}));

    btRigidBody::btRigidBodyConstructionInfo groundCI(
        0.f, m_groundState.get(), m_groundShape.get());
    groundCI.m_restitution = 0.3f;
    groundCI.m_friction    = 0.8f;
    m_ground = std::make_unique<btRigidBody>(groundCI);
    m_world->addRigidBody(m_ground.get());

    // ── Robot base (box 0.6×0.3×0.6 m, kinematic) ───────────────────────
    m_baseShape = std::make_unique<btBoxShape>(btVector3(0.3f, 0.15f, 0.3f));
    m_baseState = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), {0, 0.15f, 0}));

    btRigidBody::btRigidBodyConstructionInfo baseCI(
        60.f, m_baseState.get(), m_baseShape.get()); // 60 kg robot
    m_base = std::make_unique<btRigidBody>(baseCI);

    // Start as kinematic (operator-driven)
    m_base->setCollisionFlags(
        m_base->getCollisionFlags() |
        btCollisionObject::CF_KINEMATIC_OBJECT);
    m_base->setActivationState(DISABLE_DEACTIVATION);
    m_world->addRigidBody(m_base.get());
}

PhysicsWorld::~PhysicsWorld() {
    m_world->removeRigidBody(m_base.get());
    m_world->removeRigidBody(m_ground.get());
}

void PhysicsWorld::step(float dt) {
    // max 4 substeps, fixed internal timestep 1/240 s
    m_world->stepSimulation(dt, 4, 1.f / 240.f);
}

Vec3 PhysicsWorld::basePosition() const {
    btTransform t;
    m_base->getMotionState()->getWorldTransform(t);
    auto& o = t.getOrigin();
    return {o.x(), o.y(), o.z()};
}

void PhysicsWorld::setBaseVelocity(const Vec3& v) {
    if(m_base->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT) {
        // For kinematic bodies, directly update motion state each frame
        btTransform t;
        m_base->getMotionState()->getWorldTransform(t);
        t.setOrigin(t.getOrigin() + btVector3(v.x, v.y, v.z) * 0.016f);
        m_baseState->setWorldTransform(t);
        m_base->setWorldTransform(t);
    } else {
        m_base->setLinearVelocity({v.x, v.y, v.z});
    }
}

void PhysicsWorld::setBaseKinematic(bool kinematic) {
    if(kinematic) {
        m_base->setCollisionFlags(
            m_base->getCollisionFlags() |
            btCollisionObject::CF_KINEMATIC_OBJECT);
    } else {
        m_base->setCollisionFlags(
            m_base->getCollisionFlags() &
            ~btCollisionObject::CF_KINEMATIC_OBJECT);
        btVector3 inertia;
        m_baseShape->calculateLocalInertia(60.f, inertia);
        m_base->setMassProps(60.f, inertia);
    }
    m_base->activate(true);
}

btRigidBody* PhysicsWorld::makeRigidBody(btCollisionShape* shape,
                                          btMotionState*    state,
                                          float             mass)
{
    btVector3 inertia(0,0,0);
    if(mass > 0.f) shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, state, shape, inertia);
    return new btRigidBody(ci);
}
