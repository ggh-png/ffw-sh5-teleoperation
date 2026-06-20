#include "HandPhysics.hpp"
#include "physics/CollisionGroups.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

// ── helpers ───────────────────────────────────────────────────────────────────

// Static pointer for the pre-tick callback.  getWorldUserInfo() can be null if
// PhysicsWorld uses it internally; a static is simpler and safe for one instance.
static HandPhysics* s_preTickInstance = nullptr;

static void handPhysicsPreTick(btDynamicsWorld*, btScalar) {
    if(s_preTickInstance) s_preTickInstance->applyGripTorques();
}

static btVector3    toBt(const Vec3& v)      { return {v.x, v.y, v.z}; }
static btQuaternion toBt(const Quaternion& q) { return {q.x, q.y, q.z, q.w}; }

static bool hasPrefix(const std::string& s, const char* p) {
    return s.rfind(p, 0) == 0;
}
static Vec3 worldPos(const SceneNode* n) {
    return n->worldTransform.transformPoint({0,0,0});
}
static Quaternion worldRot(const SceneNode* n) {
    return n->worldTransform.extractRotation();
}
static Vec3 toLocal(const Quaternion& bodyRot, const Vec3& v) {
    return bodyRot.conjugate().rotate(v);
}

// ── shouldInclude ─────────────────────────────────────────────────────────────
// Only finger phalange bodies (both hands) with valid inertial data.

bool HandPhysics::shouldInclude(const SceneNode* node) {
    if(!node->inertial.valid || node->inertial.mass <= 0.f) return false;
    const auto& nm = node->name;
    bool isLeft  = hasPrefix(nm, "finger_l_link");
    bool isRight = hasPrefix(nm, "finger_r_link");
    if(!isLeft && !isRight) return false;
    // "finger_l_link" / "finger_r_link" = 13 chars; number follows immediately.
    // Links 17-20 are the 5th finger (pinky) — excluded from physics control.
    int num = std::atoi(nm.c_str() + 13);
    return num <= 16;
}

// ── buildShape ────────────────────────────────────────────────────────────────

btConvexHullShape* HandPhysics::buildShape(const SceneNode* node,
                                             const std::vector<std::string>& meshPaths) {
    if(node->meshIndex < 0 || node->meshIndex >= (int)meshPaths.size()) return nullptr;
    STLMesh mesh = STLLoader::load(meshPaths[node->meshIndex]);
    if(mesh.empty()) return nullptr;

    const Vec3 sc  = node->meshScale;
    // btMultiBody's link frame is centred at the CoM.  Mesh vertices are in
    // the body's local frame (origin = joint origin in MJCF), so subtract the
    // CoM offset to convert them into the CoM frame that Bullet expects.
    const Vec3 com = node->inertial.com;

    btConvexHullShape raw;
    for(const auto& v : mesh.vertices)
        raw.addPoint({v.x*sc.x - com.x,
                      v.y*sc.y - com.y,
                      v.z*sc.z - com.z}, false);
    raw.recalcLocalAabb();
    // Zero raw margin BEFORE btShapeHull queries support points.
    // Default = 0.04 m (4 cm) → inflates every sampled vertex by 4 cm.
    raw.setMargin(0.f);

    btShapeHull sh(&raw);
    sh.buildHull(0.f);

    auto* hull = new btConvexHullShape(
        (const btScalar*)sh.getVertexPointer(), sh.numVertices(), sizeof(btVector3));
    hull->setMargin(0.001f);  // 1 mm skin for stable GJK convergence
    return hull;
}

// ── buildHand ─────────────────────────────────────────────────────────────────

void HandPhysics::buildHand(int side, btMultiBodyDynamicsWorld* world,
                              SceneNode* palmNode,
                              const std::vector<std::string>& meshPaths) {
    auto& H = m_hands[side];

    // ── DFS: collect finger links ────────────────────────────────────────────
    {
        std::vector<std::pair<SceneNode*, int>> stack;
        for(auto it = palmNode->children.rbegin();
                 it != palmNode->children.rend(); ++it)
            if(shouldInclude(it->get()))
                stack.push_back({it->get(), -1});

        while(!stack.empty()) {
            auto [node, parentIdx] = stack.back();
            stack.pop_back();
            int myIdx = (int)H.links.size();
            const Joint& j = node->joint;
            H.links.push_back({node, parentIdx,
                                j.kp, j.damping, j.forceMin, j.forceMax});
            for(auto it = node->children.rbegin(); it != node->children.rend(); ++it)
                if(shouldInclude(it->get()))
                    stack.push_back({it->get(), myIdx});
        }
    }

    int numLinks = (int)H.links.size();
    if(numLinks == 0) {
        fprintf(stderr, "[HandPhysics] side=%d: no finger links found\n", side);
        return;
    }
    fprintf(stderr, "[HandPhysics] side=%d: building btMultiBody with %d finger links\n",
            side, numLinks);

    // ── Create btMultiBody (fixed base = palm, kinematic) ────────────────────
    H.mb = new btMultiBody(numLinks, 0.f, btVector3(0,0,0),
                           /*fixedBase*/true, /*canSleep*/false);
    H.mb->setHasSelfCollision(false);  // finger self-collision = expensive, skip
    H.mb->setLinearDamping(0.f);
    H.mb->setAngularDamping(0.05f);

    for(int i = 0; i < numLinks; ++i) {
        auto& info       = H.links[i];
        SceneNode* node  = info.node;
        int parentIdx    = info.mbLinkIdx;
        SceneNode* parentNode = (parentIdx < 0) ? palmNode : H.links[parentIdx].node;

        const float mass   = node->inertial.mass;
        const Vec3& Id     = node->inertial.diagInertia;
        // 1e-3 kg·m² floor: with kp=20, ωn=sqrt(20/1e-3)=141 rad/s,
        // dt×ωn=0.28 < 2 → explicit-Euler stable; ζ≈3.5 → no oscillation.
        btVector3 inertia(std::max(1e-3f, Id.x),
                          std::max(1e-3f, Id.y),
                          std::max(1e-3f, Id.z));

        // Pivot world position (joint origin)
        Mat4 Tpivot     = parentNode->worldTransform * node->localTransform();
        Vec3 pivotW     = Tpivot.transformPoint({0,0,0});
        Vec3 axisW      = Tpivot.transformDir(node->joint.axis).normalized();

        Quaternion Rp   = worldRot(parentNode);
        Quaternion Rc   = worldRot(node);
        Vec3 parentComW = parentNode->worldTransform.transformPoint(parentNode->inertial.com);
        Vec3 thisComW   = node->worldTransform.transformPoint(node->inertial.com);

        Quaternion rotPtoC          = (Rc.conjugate() * Rp).normalized();
        Vec3 parentComToPivot_local = toLocal(Rp, pivotW - parentComW);
        Vec3 pivotToChildCom_local  = toLocal(Rc, thisComW - pivotW);
        Vec3 axisInChild            = toLocal(Rc, axisW);

        H.mb->setupRevolute(i, mass, inertia, parentIdx,
                             toBt(rotPtoC),
                             btVector3(axisInChild.x, axisInChild.y, axisInChild.z),
                             btVector3(parentComToPivot_local.x,
                                       parentComToPivot_local.y,
                                       parentComToPivot_local.z),
                             btVector3(pivotToChildCom_local.x,
                                       pivotToChildCom_local.y,
                                       pivotToChildCom_local.z),
                             /*disableParentCollision*/true);
    }

    H.mb->finalizeMultiDof();
    for(int i = 0; i < numLinks; ++i) {
        H.mb->setJointPos(i, H.links[i].node->joint.value);
        H.mb->setJointVel(i, 0.f);
    }

    // Set initial palm transform
    Quaternion pr = worldRot(palmNode);
    Vec3       pp = worldPos(palmNode);
    H.mb->setBaseWorldTransform(btTransform(toBt(pr), toBt(pp)));

    world->addMultiBody(H.mb);

    // ── Collision shapes and link colliders ───────────────────────────────────
    H.colliders.emplace_back(nullptr);  // base placeholder

    // Links are ordered by DFS: thumb (0-3), index (4-7), middle (8-11), ring (12-15).
    // Encode (side, chain, localIdx) in userIndex so FingerChainFilter can block
    // intra-chain pairs while allowing cross-chain (inter-finger) collision.
    //   userIndex = side*40 + chainIdx*10 + localIdx
    static constexpr int kLinksPerChain = 4;

    for(int i = 0; i < numLinks; ++i) {
        SceneNode* node = H.links[i].node;
        btConvexHullShape* shape = buildShape(node, meshPaths);
        H.shapes.emplace_back(shape);

        auto* col = new btMultiBodyLinkCollider(H.mb, i);
        if(shape) {
            col->setCollisionShape(shape);
            // Initial transform: CoM world position (Featherstone uses CoM frames)
            Vec3 comW = node->worldTransform.transformPoint(node->inertial.com);
            col->setWorldTransform(btTransform(toBt(worldRot(node)), toBt(comW)));
            col->setFriction(10.0f);
            col->setRestitution(0.0f);
        }
        int chainIdx = i / kLinksPerChain;
        int localIdx = i % kLinksPerChain;
        col->setUserIndex(side * 40 + chainIdx * 10 + localIdx);

        col->setCcdMotionThreshold(0.005f);
        col->setCcdSweptSphereRadius(0.010f);
        H.colliders.emplace_back(col);
        H.mb->getLink(i).m_collider = col;
        world->addCollisionObject(col, ColGroup::HAND, ColGroup::MASK_HAND);
    }

    // ── Joint limits ─────────────────────────────────────────────────────────
    for(int i = 0; i < numLinks; ++i) {
        const Joint& j = H.links[i].node->joint;
        if(!j.hasLimits) continue;
        auto* lim = new btMultiBodyJointLimitConstraint(H.mb, i, j.limitLo, j.limitHi);
        H.limits.emplace_back(lim);
        world->addMultiBodyConstraint(lim);
    }

    fprintf(stderr, "[HandPhysics] side=%d: %d links, %d limits, built OK\n",
            side, numLinks, (int)H.limits.size());
}

// ── build ─────────────────────────────────────────────────────────────────────

void HandPhysics::build(btMultiBodyDynamicsWorld* world,
                          SceneNode* palmL, SceneNode* palmR,
                          const std::vector<std::string>& meshPaths) {
    m_world = world;
    if(palmL) buildHand(0, world, palmL, meshPaths);
    if(palmR) buildHand(1, world, palmR, meshPaths);

    // Register per-substep PD torque callback.
    // addJointTorque is cleared each substep by Bullet, so calling it once per
    // frame (60 Hz) gives effective kp at frame rate → dt×ωn=2.35 → unstable.
    // Running it inside the pre-tick callback keeps effective dt=0.002 s (500 Hz)
    // → dt×ωn=0.28 → stable.
    // Broadphase filter: block intra-chain pairs (explosive impulses), allow cross-chain.
    world->getBroadphase()->getOverlappingPairCache()
         ->setOverlapFilterCallback(&m_chainFilter);

    s_preTickInstance = this;
    world->setInternalTickCallback(handPhysicsPreTick, nullptr, /*isPreTick=*/true);
}

// ── setPalmTransforms ─────────────────────────────────────────────────────────

void HandPhysics::setPalmTransforms(const Mat4& palmLWorld, const Mat4& palmRWorld) {
    const Mat4* mats[2] = {&palmLWorld, &palmRWorld};
    for(int s = 0; s < 2; ++s) {
        if(!m_hands[s].mb) continue;
        Quaternion rot = mats[s]->extractRotation();
        Vec3       pos = mats[s]->transformPoint({0,0,0});
        m_hands[s].mb->setBaseWorldTransform(btTransform(toBt(rot), toBt(pos)));
        if(!m_hands[s].colliders.empty() && m_hands[s].colliders[0])
            m_hands[s].colliders[0]->setWorldTransform(btTransform(toBt(rot), toBt(pos)));
    }
}

// ── applyGripTorques ──────────────────────────────────────────────────────────
// PD servo via addJointTorque — called per physics substep (500 Hz) via the
// Bullet pre-tick callback registered in build().
//
// Uses the public members kp / kd / forceMax (tunable from UI at runtime).
// Stability at dt_sub=0.002 s, I_min=1e-3 kg·m²:
//   safe range: kp ∈ [0, ~400], kd ∈ [0, 0.9]
//   kp=3, kd=0.5 → ωn=55 rad/s, ζ=4  → heavily overdamped, no divergence

void HandPhysics::applyGripTorques() {
    for(int s = 0; s < 2; ++s) {
        auto& H = m_hands[s];
        if(!H.mb) continue;
        for(int i = 0; i < (int)H.links.size(); ++i) {
            float q      = H.mb->getJointPos(i);
            float qdot   = H.mb->getJointVel(i);
            float target = H.links[i].node->joint.value;

            float tau = kp * (target - q) - kd * qdot;
            tau = std::max(-forceMax, std::min(forceMax, tau));

            H.mb->addJointTorque(i, tau);
        }
    }
}

// ── syncToFK ─────────────────────────────────────────────────────────────────
// After physics step: write Featherstone joint positions back to SceneNode.
// The renderer then shows fingers at their physically simulated positions
// (i.e. fingers resist the object rather than passing through it).

void HandPhysics::syncToFK() {
    for(int s = 0; s < 2; ++s) {
        auto& H = m_hands[s];
        if(!H.mb) continue;
        for(int i = 0; i < (int)H.links.size(); ++i) {
            float q = H.mb->getJointPos(i);
            auto* node = H.links[i].node;
            if(node->joint.hasLimits) {
                q = std::max(node->joint.limitLo, std::min(node->joint.limitHi, q));
            }
            node->joint.value    = q;
            node->joint.valueSag = 0.f;
        }
    }
}

// ── clear ─────────────────────────────────────────────────────────────────────

void HandPhysics::clear(btMultiBodyDynamicsWorld* world) {
    s_preTickInstance = nullptr;
    world->setInternalTickCallback(nullptr, nullptr, /*isPreTick=*/true);
    world->getBroadphase()->getOverlappingPairCache()->setOverlapFilterCallback(nullptr);

    for(auto& H : m_hands) {
        for(auto& lim : H.limits)
            if(lim) world->removeMultiBodyConstraint(lim.get());
        H.limits.clear();
        for(auto& col : H.colliders)
            if(col) world->removeCollisionObject(col.get());
        H.colliders.clear();
        H.shapes.clear();
        if(H.mb) { world->removeMultiBody(H.mb); delete H.mb; H.mb = nullptr; }
        H.links.clear();
    }
    m_world = nullptr;
}
