#include "RobotCollider.hpp"
#include "physics/CollisionGroups.hpp"
#include "io/STLLoader.hpp"
#include <cstdio>

// ── Helpers ───────────────────────────────────────────────────────────────────


btTransform RobotCollider::fromMat4(const Mat4& m) {
    Quaternion q = m.extractRotation();
    return btTransform(
        btQuaternion(q.x, q.y, q.z, q.w),
        btVector3(m.at(0,3), m.at(1,3), m.at(2,3)));
}

// ── Build ─────────────────────────────────────────────────────────────────────

void RobotCollider::build(btMultiBodyDynamicsWorld* world,
                           const std::vector<SceneNode*>& nodes,
                           const std::vector<std::string>& meshPaths)
{
    for(auto* node : nodes) {
        if(!node) continue;
        if(node->meshIndex < 0 || node->meshIndex >= (int)meshPaths.size()) continue;

        // Finger phalanges (finger_l/r_link1-16) are owned by HandPhysics as
        // dynamic Featherstone btMultiBodyLinkCollider bodies.  Registering them
        // here as kinematic btRigidBody too would create two bodies at the same
        // position, causing explosive constraint forces.
        {
            const auto& nm = node->name;
            if(nm.rfind("finger_l_link", 0) == 0 || nm.rfind("finger_r_link", 0) == 0) continue;
        }

        STLMesh stl = STLLoader::load(meshPaths[node->meshIndex]);
        if(stl.empty()) continue;

        const Vec3 sc = node->meshScale;

        // Build raw convex hull from all mesh vertices (with applied per-node scale)
        btConvexHullShape rawHull;
        for(const auto& v : stl.vertices)
            rawHull.addPoint(btVector3(v.x*sc.x, v.y*sc.y, v.z*sc.z), false);
        rawHull.recalcLocalAabb();
        rawHull.setMargin(0.f);  // zero before btShapeHull queries — default 4 cm inflates hull

        // Reduce to a simplified hull for performance (~100 verts typical result)
        btShapeHull shapeHull(&rawHull);
        shapeHull.buildHull(0.f);

        auto& lb = m_links.emplace_back();
        lb.node  = node;
        lb.shape = std::make_unique<btConvexHullShape>(
            (const btScalar*)shapeHull.getVertexPointer(),
            shapeHull.numVertices(),
            sizeof(btVector3));
        lb.shape->setMargin(0.001f);

        btTransform initT = fromMat4(node->worldTransform);
        lb.state = std::make_unique<btDefaultMotionState>(initT);

        btRigidBody::btRigidBodyConstructionInfo ci(
            0.f, lb.state.get(), lb.shape.get());
        ci.m_restitution = 0.1f;
        ci.m_friction    = 0.7f;
        lb.body = std::make_unique<btRigidBody>(ci);
        lb.body->setCollisionFlags(
            lb.body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        lb.body->setActivationState(DISABLE_DEACTIVATION);

        // ROBOT_LINK with mask=0: invisible to Bullet's broadphase.
        // Arm↔env contacts are tested explicitly via findStaticPenetrations
        // (contactPairTest bypasses the broadphase filter), so these bodies
        // will never inadvertently push the can through normal physics.
        world->addRigidBody(lb.body.get(), ColGroup::ROBOT_LINK, ColGroup::MASK_ROBOT_LINK);
    }

    printf("[RobotCollider] built %d link bodies\n", (int)m_links.size());
}

// ── Per-frame update ──────────────────────────────────────────────────────────

void RobotCollider::update() {
    for(auto& lb : m_links) {
        if(!lb.node || !lb.body) continue;
        btTransform t = fromMat4(lb.node->worldTransform);
        lb.state->setWorldTransform(t);
        lb.body->setWorldTransform(t);
    }
}

// ── Cleanup ───────────────────────────────────────────────────────────────────

void RobotCollider::clear(btMultiBodyDynamicsWorld* world) {
    for(auto& lb : m_links)
        if(lb.body) world->removeRigidBody(lb.body.get());
    m_links.clear();
}

// ── Penetration detection ─────────────────────────────────────────────────────

SceneNode* RobotCollider::nodeForBody(const btCollisionObject* body) const {
    for(const auto& lb : m_links)
        if(lb.body.get() == body) return lb.node;
    return nullptr;
}

std::vector<btRigidBody*> RobotCollider::bodiesForPrefix(const std::string& prefix) const {
    std::vector<btRigidBody*> result;
    for(const auto& lb : m_links)
        if(lb.node && lb.body &&
           lb.node->name.rfind(prefix, 0) == 0)
            result.push_back(lb.body.get());
    return result;
}

btRigidBody* RobotCollider::bodyForName(const std::string& name) const {
    for(const auto& lb : m_links)
        if(lb.node && lb.node->name == name && lb.body)
            return lb.body.get();
    return nullptr;
}

namespace {
// Callback for contactPairTest: collects penetrating contacts between a robot link (A)
// and an env body (B).  m_normalWorldOnB points from B toward A = push-out direction for A.
struct PenCB : public btCollisionWorld::ContactResultCallback {
    SceneNode* node = nullptr;
    std::vector<RobotCollider::StaticPenetration>* out = nullptr;

    btScalar addSingleResult(btManifoldPoint& cp,
                              const btCollisionObjectWrapper*, int, int,
                              const btCollisionObjectWrapper*, int, int) override {
        if(cp.getDistance() >= 0.f) return 0.f;  // not penetrating
        btVector3 n  = cp.m_normalWorldOnB;       // push robot link out of env
        btVector3 pt = cp.getPositionWorldOnA();   // point on robot link surface
        out->push_back({node,
                        Vec3{n.x(),  n.y(),  n.z()},
                        -cp.getDistance(),
                        Vec3{pt.x(), pt.y(), pt.z()}});
        return 0.f;
    }
};
}  // namespace

std::vector<RobotCollider::StaticPenetration>
RobotCollider::findStaticPenetrations(btMultiBodyDynamicsWorld* world,
                                       const std::vector<btRigidBody*>& envBodies) const {
    std::vector<StaticPenetration> result;
    for(const auto& lb : m_links) {
        if(!lb.body || !lb.node) continue;
        for(auto* env : envBodies) {
            // Skip nullptr and self-tests: a body testing against its own btRigidBody
            // produces an inside-out contact with large negative depth, causing the
            // Jacobian correction to fire with a bogus displacement.
            if(!env || env == lb.body.get()) continue;
            PenCB cb;
            cb.node = lb.node;
            cb.out  = &result;
            world->contactPairTest(lb.body.get(), env, cb);
        }
    }
    return result;
}
