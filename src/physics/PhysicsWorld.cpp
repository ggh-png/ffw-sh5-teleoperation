#include "PhysicsWorld.hpp"
#include "physics/CollisionGroups.hpp"
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio>

PhysicsWorld::PhysicsWorld(const PhysicsOptions& opts) {
    m_timestep = std::max(1.f/1000.f, std::min(1.f/60.f, opts.timestep));
    m_impratio = std::max(1.f, opts.impratio);

    m_colConfig  = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_colConfig.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    // btMultiBodyConstraintSolver handles both regular and Featherstone constraints
    m_solver     = std::make_unique<btMultiBodyConstraintSolver>();

    m_world = std::make_unique<btMultiBodyDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(),
        m_solver.get(), m_colConfig.get());
    m_world->setGravity({0, opts.gravity, 0});

    // ── Solver settings ───────────────────────────────────────────────────────
    // PGS + Split Impulse (Bullet) + Baumgarte ERP (ODE) +
    // Warm starting (Roblox) + Linear slop (DigiPen Slop term)
    {
        btContactSolverInfo& info = m_world->getSolverInfo();
        info.m_numIterations                    = 50;
        info.m_splitImpulse                     = true;
        info.m_splitImpulsePenetrationThreshold = -0.02f;
        info.m_erp                              = 0.8f;
        info.m_erp2                             = 0.8f;
        info.m_warmstartingFactor               = 0.85f;
        info.m_linearSlop                       = 0.001f;
        info.m_globalCfm                        = 1e-5f;
    }

    // ── Ground plane ──────────────────────────────────────────────────────────
    m_groundShape = std::make_unique<btStaticPlaneShape>(btVector3(0,1,0), 0.f);
    m_groundState = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), {0,0,0}));
    btRigidBody::btRigidBodyConstructionInfo gCI(
        0.f, m_groundState.get(), m_groundShape.get());
    gCI.m_restitution = 0.1f;
    gCI.m_friction    = 0.9f;
    m_ground = std::make_unique<btRigidBody>(gCI);
    m_world->addRigidBody(m_ground.get(), ColGroup::ENV, ColGroup::MASK_ENV);

    // ── Robot mobile base (kinematic box) ─────────────────────────────────────
    m_baseShape = std::make_unique<btBoxShape>(btVector3(0.3f, 0.15f, 0.3f));
    m_baseState = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), {0, 0.15f, 0}));
    btRigidBody::btRigidBodyConstructionInfo bCI(
        0.f, m_baseState.get(), m_baseShape.get());
    m_base = std::make_unique<btRigidBody>(bCI);
    m_base->setCollisionFlags(
        m_base->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    m_base->setActivationState(DISABLE_DEACTIVATION);
    m_world->addRigidBody(m_base.get(), ColGroup::ROBOT_BASE, ColGroup::MASK_ROBOT_BASE);
}

PhysicsWorld::~PhysicsWorld() {
    for(auto& ob : m_objects)    m_world->removeRigidBody(ob.body.get());
    for(auto& sb : m_staticBoxes) m_world->removeRigidBody(sb.body.get());
    m_world->removeRigidBody(m_base.get());
    m_world->removeRigidBody(m_ground.get());
}

// ── step ──────────────────────────────────────────────────────────────────────

void PhysicsWorld::step(float dt) {
    int maxSub = std::max(20, (int)std::ceil(dt / m_timestep) + 2);
    m_world->stepSimulation(dt, maxSub, m_timestep);
}

// ── Mobile base ───────────────────────────────────────────────────────────────

Vec3 PhysicsWorld::basePosition() const {
    btTransform t;
    m_base->getMotionState()->getWorldTransform(t);
    auto& o = t.getOrigin();
    return {o.x(), o.y(), o.z()};
}

Quaternion PhysicsWorld::baseOrientation() const {
    btTransform t;
    m_base->getMotionState()->getWorldTransform(t);
    auto q = t.getRotation();
    return Quaternion{q.w(), q.x(), q.y(), q.z()};
}

namespace {
struct BaseSweepCB : btCollisionWorld::ClosestConvexResultCallback {
    const btCollisionObject* self;
    const btCollisionObject* ground;
    BaseSweepCB(const btVector3& f, const btVector3& t,
                const btCollisionObject* s, const btCollisionObject* g)
        : ClosestConvexResultCallback(f, t), self(s), ground(g) {}

    btScalar addSingleResult(btCollisionWorld::LocalConvexResult& r,
                             bool normalInWorldSpace) override {
        if(r.m_hitCollisionObject == self)   return 1.f;
        if(r.m_hitCollisionObject == ground) return 1.f;
        int flags = r.m_hitCollisionObject->getCollisionFlags();
        if(flags & btCollisionObject::CF_KINEMATIC_OBJECT) return 1.f;
        const btRigidBody* rb = btRigidBody::upcast(r.m_hitCollisionObject);
        if(rb && rb->getMass() > 0.f) return 1.f;
        return ClosestConvexResultCallback::addSingleResult(r, normalInWorldSpace);
    }
};
}

bool PhysicsWorld::setBaseVelocity(const Vec3& robotLocalVel, float yawRate, float dt) {
    btTransform currentT;
    m_base->getMotionState()->getWorldTransform(currentT);

    m_baseYaw += yawRate * dt;
    btQuaternion yawQ(btVector3(0,1,0), m_baseYaw);
    btTransform newT = currentT;
    newT.setRotation(yawQ);

    float cy = std::cos(m_baseYaw), sy = std::sin(m_baseYaw);
    btVector3 worldVel(
        robotLocalVel.x * cy + robotLocalVel.z * sy,
        0.f,
        -robotLocalVel.x * sy + robotLocalVel.z * cy);
    worldVel *= dt;

    btVector3 fromPt = currentT.getOrigin();
    btVector3 toPt   = fromPt + worldVel;
    newT.setOrigin(toPt);

    btTransform sweepFrom = currentT; sweepFrom.setRotation(yawQ);
    BaseSweepCB cb(fromPt, toPt, m_base.get(), m_ground.get());
    m_world->convexSweepTest(m_baseShape.get(), sweepFrom, newT, cb, 0.01f);

    if(cb.hasHit()) {
        btVector3 n = cb.m_hitNormalWorld.normalized();
        float proj  = worldVel.dot(n);
        if(proj < 0.f) worldVel -= n * proj;
        toPt = fromPt + worldVel;
        newT.setOrigin(toPt);
    }

    m_baseState->setWorldTransform(newT);
    m_base->setWorldTransform(newT);
    return !cb.hasHit();
}

void PhysicsWorld::setBaseKinematic(bool kinematic) {
    if(kinematic) {
        m_base->setCollisionFlags(
            m_base->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    } else {
        m_base->setCollisionFlags(
            m_base->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
        m_base->setMassProps(60.f, {0,0,0});
    }
    m_base->activate(true);
}

bool PhysicsWorld::isGrounded() const {
    Vec3 pos = basePosition();
    return pos.y <= 0.155f;
}

// ── handNearestDist ───────────────────────────────────────────────────────────

float PhysicsWorld::handNearestDist(int side) const {
    if(side < 0 || side > 1) return 1e9f;
    float minDist = 1e9f;
    const Vec3& hp = m_palmPos[side];
    for(const auto& obj : m_objects) {
        if(obj.graspedBy >= 0) continue;
        btTransform t;
        obj.body->getMotionState()->getWorldTransform(t);
        auto& o = t.getOrigin();
        float dx = o.x()-hp.x, dy = o.y()-hp.y, dz = o.z()-hp.z;
        float d = std::sqrt(dx*dx+dy*dy+dz*dz) - obj.radius;
        if(d < minDist) minDist = d;
    }
    return minDist;
}

// ── Dynamic cylinders ─────────────────────────────────────────────────────────

int PhysicsWorld::addCylinder(float r, float halfH, float mass,
                               const Vec3& pos, const Vec3& color) {
    auto& obj = m_objects.emplace_back();
    obj.radius=r; obj.halfHeight=halfH; obj.mass=mass; obj.color=color;
    obj.shape = std::make_unique<btCylinderShape>(btVector3(r,halfH,r));
    obj.state = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), btVector3(pos.x,pos.y,pos.z)));
    btVector3 inertia(0,0,0);
    obj.shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, obj.state.get(),
                                                obj.shape.get(), inertia);
    ci.m_restitution    = 0.1f / std::sqrt(m_impratio);
    ci.m_friction       = 0.5f;
    ci.m_linearDamping  = 0.05f;
    ci.m_angularDamping = 0.1f;
    obj.body = std::make_unique<btRigidBody>(ci);
    obj.body->setActivationState(DISABLE_DEACTIVATION);
    obj.body->setCcdMotionThreshold(0.01f);
    obj.body->setCcdSweptSphereRadius(0.015f);
    m_world->addRigidBody(obj.body.get(), ColGroup::OBJ, ColGroup::MASK_OBJ);
    return (int)m_objects.size() - 1;
}

// ── Grasping ──────────────────────────────────────────────────────────────────

void PhysicsWorld::applyGripForce(int side, float gripStrength,
                                   const Vec3& palmPos, const Quaternion& palmRot,
                                   float dt) {
    m_palmPosPrev[side] = m_palmPos[side];
    m_palmPos[side]     = palmPos;

    btQuaternion bRot(palmRot.x, palmRot.y, palmRot.z, palmRot.w);
    btTransform palmT(bRot, btVector3(palmPos.x, palmPos.y, palmPos.z));

    for(auto& obj : m_objects) {
        if(obj.mouseDrag) continue;
        if(obj.graspedBy >= 0 && obj.graspedBy != side) continue;

        if(gripStrength < 0.05f) {
            if(obj.graspedBy == side) {
                obj.body->setCollisionFlags(
                    obj.body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
                btVector3 inertia;
                obj.shape->calculateLocalInertia(obj.mass, inertia);
                obj.body->setMassProps(obj.mass, inertia);
                obj.body->forceActivationState(ACTIVE_TAG);
                obj.body->clearForces();
                if(dt > 1e-5f) {
                    Vec3 dp = {
                        m_palmPos[side].x - m_palmPosPrev[side].x,
                        m_palmPos[side].y - m_palmPosPrev[side].y,
                        m_palmPos[side].z - m_palmPosPrev[side].z
                    };
                    Vec3 tv = {dp.x/dt, dp.y/dt, dp.z/dt};
                    float spd2 = tv.x*tv.x + tv.y*tv.y + tv.z*tv.z;
                    if(spd2 > 25.f) {
                        float inv = 5.f / std::sqrt(spd2);
                        tv = {tv.x*inv, tv.y*inv, tv.z*inv};
                    }
                    obj.body->setLinearVelocity(btVector3(tv.x, tv.y, tv.z));
                } else {
                    obj.body->setLinearVelocity({0,0,0});
                }
                obj.body->setAngularVelocity({0,0,0});
                obj.body->activate(true);
                obj.graspedBy = -1;
            }
            continue;
        }

        if(obj.graspedBy == side) continue;

        // Cylinder-surface distance from palm centre
        btTransform objT;
        obj.body->getMotionState()->getWorldTransform(objT);
        btVector3 lp = objT.inverse() * btVector3(palmPos.x, palmPos.y, palmPos.z);
        float lat = std::sqrt(lp.x()*lp.x() + lp.z()*lp.z());
        float ay  = std::fabs(lp.y());
        float surfDist;
        if(lat <= obj.radius && ay <= obj.halfHeight)
            surfDist = -std::min(obj.radius-lat, obj.halfHeight-ay);
        else if(ay > obj.halfHeight && lat <= obj.radius)
            surfDist = ay - obj.halfHeight;
        else if(lat > obj.radius && ay <= obj.halfHeight)
            surfDist = lat - obj.radius;
        else {
            float ex = lat-obj.radius, ey = ay-obj.halfHeight;
            surfDist = std::sqrt(ex*ex+ey*ey);
        }
        if(surfDist > kGraspRadius) continue;

        obj.graspedBy = side;
        obj.body->setCollisionFlags(
            obj.body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        obj.body->setActivationState(DISABLE_DEACTIVATION);
        obj.graspRelTransform = palmT.inverse() * objT;
    }
}

void PhysicsWorld::updateGraspedObjects(int side, const Vec3& palmPos,
                                         const Quaternion& palmRot) {
    btQuaternion bRot(palmRot.x, palmRot.y, palmRot.z, palmRot.w);
    btTransform palmT(bRot, btVector3(palmPos.x, palmPos.y, palmPos.z));
    for(auto& obj : m_objects) {
        if(obj.graspedBy != side || obj.mouseDrag) continue;
        btTransform newT = palmT * obj.graspRelTransform;
        obj.state->setWorldTransform(newT);
        obj.body->setWorldTransform(newT);
        obj.body->clearForces();
        obj.body->setLinearVelocity({0,0,0});
        obj.body->setAngularVelocity({0,0,0});
    }
}

// ── Mouse drag ────────────────────────────────────────────────────────────────

void PhysicsWorld::beginMouseDrag(int idx) {
    if(idx < 0 || idx >= (int)m_objects.size()) return;
    auto& obj = m_objects[idx];
    if(obj.graspedBy >= 0) return;
    obj.mouseDrag = true;
    obj.body->setCollisionFlags(
        obj.body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    obj.body->setActivationState(DISABLE_DEACTIVATION);
}

void PhysicsWorld::setMouseDragPosition(int idx, const Vec3& pos) {
    if(idx < 0 || idx >= (int)m_objects.size()) return;
    btTransform t(btQuaternion::getIdentity(), btVector3(pos.x,pos.y,pos.z));
    m_objects[idx].body->getMotionState()->setWorldTransform(t);
    m_objects[idx].body->setWorldTransform(t);
    m_objects[idx].body->clearForces();
    m_objects[idx].body->setLinearVelocity({0,0,0});
    m_objects[idx].body->setAngularVelocity({0,0,0});
}

void PhysicsWorld::endMouseDrag(int idx) {
    if(idx < 0 || idx >= (int)m_objects.size()) return;
    auto& obj = m_objects[idx];
    obj.mouseDrag = false;
    obj.body->setCollisionFlags(
        obj.body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
    obj.body->setActivationState(ACTIVE_TAG);
    btVector3 inertia;
    obj.shape->calculateLocalInertia(obj.mass, inertia);
    obj.body->setMassProps(obj.mass, inertia);
    obj.body->clearForces();
    obj.body->setLinearVelocity({0,0,0});
    obj.body->activate(true);
}

// ── Object states ─────────────────────────────────────────────────────────────

std::vector<PhysicsWorld::ObjState> PhysicsWorld::objectStates() const {
    std::vector<ObjState> s;
    for(const auto& obj : m_objects) {
        btTransform t;
        obj.body->getMotionState()->getWorldTransform(t);
        auto& o=t.getOrigin(); auto q=t.getRotation();
        s.push_back({{o.x(),o.y(),o.z()},{q.w(),q.x(),q.y(),q.z()},
                     obj.color,obj.radius,obj.halfHeight});
    }
    return s;
}

// ── Static boxes ──────────────────────────────────────────────────────────────

int PhysicsWorld::addStaticBox(const Vec3& halfE, const Vec3& pos, const Vec3& color) {
    auto& obj = m_staticBoxes.emplace_back();
    obj.pos=pos; obj.halfExtents=halfE; obj.color=color;
    obj.shape = std::make_unique<btBoxShape>(btVector3(halfE.x,halfE.y,halfE.z));
    obj.state = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), btVector3(pos.x,pos.y,pos.z)));
    btRigidBody::btRigidBodyConstructionInfo ci(0.f,obj.state.get(),obj.shape.get());
    ci.m_restitution = 0.1f / std::sqrt(m_impratio);
    ci.m_friction    = 0.8f;
    obj.body = std::make_unique<btRigidBody>(ci);
    m_world->addRigidBody(obj.body.get(), ColGroup::ENV, ColGroup::MASK_TABLE);
    return (int)m_staticBoxes.size()-1;
}

std::vector<PhysicsWorld::StaticBoxState> PhysicsWorld::staticBoxStates() const {
    std::vector<StaticBoxState> s;
    for(const auto& obj : m_staticBoxes)
        s.push_back({obj.pos, obj.halfExtents, obj.color});
    return s;
}

// ── Scene spawning ────────────────────────────────────────────────────────────

void PhysicsWorld::spawnObjects(const std::vector<ObjectDesc>& descs) {
    for(const auto& d : descs) {
        if(d.type == ObjectDesc::Type::Box) {
            addStaticBox(d.halfExtents, d.pos, d.color);
        } else if(d.type == ObjectDesc::Type::Cylinder) {
            addCylinder(d.radius, d.halfHeight, d.mass, d.pos, d.color);
        }
    }
}

// ── Ray picking ───────────────────────────────────────────────────────────────

int PhysicsWorld::pickObject(const Vec3& rayOrigin, const Vec3& rayDir) const {
    btVector3 from(rayOrigin.x, rayOrigin.y, rayOrigin.z);
    btVector3 to(rayOrigin.x + rayDir.x * 100.f,
                 rayOrigin.y + rayDir.y * 100.f,
                 rayOrigin.z + rayDir.z * 100.f);
    btCollisionWorld::ClosestRayResultCallback cb(from, to);
    m_world->rayTest(from, to, cb);
    if(!cb.hasHit()) return -1;
    for(int i = 0; i < (int)m_objects.size(); ++i)
        if(m_objects[i].body.get() == cb.m_collisionObject) return i;
    return -1;
}

btRigidBody* PhysicsWorld::makeRigidBody(btCollisionShape* shape,
                                          btMotionState* ms, float mass) {
    btVector3 inertia(0,0,0);
    if(mass > 0.f) shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, ms, shape, inertia);
    return new btRigidBody(ci);
}
