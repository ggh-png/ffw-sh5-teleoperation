#include "ArticulatedRobot.hpp"
#include "physics/CollisionGroups.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <map>

// ── Helpers ───────────────────────────────────────────────────────────────────

static btVector3  toBt(const Vec3& v) { return {v.x, v.y, v.z}; }
static btQuaternion toBt(const Quaternion& q) { return {q.x, q.y, q.z, q.w}; } // Bullet XYZW

// Extract world-space position of the node origin from its worldTransform.
static Vec3 worldPos(const SceneNode* n) {
    return n->worldTransform.transformPoint({0,0,0});
}

// Extract world-space rotation from worldTransform.
static Quaternion worldRot(const SceneNode* n) {
    return n->worldTransform.extractRotation();
}

// Rotate a world-space vector into a body's local frame.
// (inverse of bodyRot, which maps local→world)
static Vec3 toLocal(const Quaternion& bodyRot, const Vec3& v) {
    return bodyRot.conjugate().rotate(v);
}

// ── shouldInclude ──────────────────────────────────────────────────────────────
// Include arm links and hand links that have valid inertial data.
// Wheel joints, head joints, lift, and base links are NOT included (they remain
// kinematic and are controlled directly by the user).

static bool hasPrefix(const std::string& s, const char* p) {
    return s.rfind(p, 0) == 0;
}

bool ArticulatedRobot::shouldInclude(const SceneNode* node) {
    if(!node->inertial.valid || node->inertial.mass <= 0.f) return false;
    const auto& nm = node->name;
    return hasPrefix(nm, "arm_l_link") ||
           hasPrefix(nm, "arm_r_link") ||
           hasPrefix(nm, "hx5_l")      ||
           hasPrefix(nm, "hx5_r")      ||
           hasPrefix(nm, "finger_l_")  ||
           hasPrefix(nm, "finger_r_")  ||
           hasPrefix(nm, "thumb_l_")   ||
           hasPrefix(nm, "thumb_r_");
}

// ── buildShape ────────────────────────────────────────────────────────────────

btConvexHullShape* ArticulatedRobot::buildShape(const SceneNode* node,
                                                  const std::vector<std::string>& meshPaths) {
    if(node->meshIndex < 0 || node->meshIndex >= (int)meshPaths.size()) return nullptr;
    STLMesh mesh = STLLoader::load(meshPaths[node->meshIndex]);
    if(mesh.empty()) return nullptr;

    const Vec3 sc = node->meshScale;
    btConvexHullShape rawHull;
    for(const auto& v : mesh.vertices)
        rawHull.addPoint({v.x*sc.x, v.y*sc.y, v.z*sc.z}, false);
    rawHull.recalcLocalAabb();

    // Simplify to ~100 vertex hull for performance
    btShapeHull sh(&rawHull);
    sh.buildHull(0.001f);

    auto* hull = new btConvexHullShape(
        (const btScalar*)sh.getVertexPointer(), sh.numVertices(), sizeof(btVector3));
    hull->setMargin(0.001f);
    return hull;
}

// ── addCollider ───────────────────────────────────────────────────────────────

void ArticulatedRobot::addCollider(btConvexHullShape* shape, int linkIdx, const Mat4& worldT) {
    auto* col = new btMultiBodyLinkCollider(m_mb, linkIdx);
    if(shape) {
        col->setCollisionShape(shape);
        // Position: use world transform of the link origin
        Quaternion rot = worldT.extractRotation();
        Vec3       pos = worldT.transformPoint({0,0,0});
        btTransform t(toBt(rot), toBt(pos));
        col->setWorldTransform(t);
        // Friction from MJCF default (geomFriction of 0.7 if not set)
        col->setFriction(0.7f);
        col->setRestitution(0.05f);
    }
    m_colliders.emplace_back(col);
    // Register arm/hand links as ROBOT_LINK — physics-invisible to can in broad phase
    // but Featherstone contact solver still handles them via the multi-body constraint.
    // Hand links (hx5, finger) use HAND group so they CAN push the can.
    m_world->addCollisionObject(col, ColGroup::ROBOT_LINK, ColGroup::ALL);
    m_mb->getLink(std::max(linkIdx, 0)).m_collider = col;
}

// ── build ─────────────────────────────────────────────────────────────────────

void ArticulatedRobot::build(btMultiBodyDynamicsWorld* world,
                              SceneNode*               armBaseNode,
                              const std::vector<std::string>& meshPaths) {
    m_world = world;

    // ── Step 1: DFS collect links with correct parent tracking ───────────────
    {
        std::vector<std::pair<SceneNode*, int>> dfsStack;
        for(auto it = armBaseNode->children.rbegin();
                 it != armBaseNode->children.rend(); ++it)
            if(shouldInclude(it->get()))
                dfsStack.push_back({it->get(), -1});

        while(!dfsStack.empty()) {
            auto [node, parentMbIdx] = dfsStack.back();
            dfsStack.pop_back();
            int myIdx = (int)m_links.size();
            const Joint& j = node->joint;
            m_links.push_back({node, parentMbIdx, j.kp, j.damping,
                               j.forceMin, j.forceMax, j.value,
                               j.type == JointType::Revolute});
            for(auto it = node->children.rbegin(); it != node->children.rend(); ++it)
                if(shouldInclude(it->get()))
                    dfsStack.push_back({it->get(), myIdx});
        }
    }
    int numLinks = (int)m_links.size();
    fprintf(stderr, "[ArticulatedRobot] Building btMultiBody: %d links from '%s'\n",
            numLinks, armBaseNode->name.c_str());
    for(int i = 0; i < numLinks; ++i) {
        const auto& li = m_links[i];
        fprintf(stderr, "  [%2d] %-30s parent=%-3d  kp=%7.1f  damp=%.3f  mass=%.4f  isRev=%d\n",
                i, li.node->name.c_str(), li.mbLinkIdx,
                li.kp, li.damping,
                li.node->inertial.mass, (int)li.isRevolute);
    }
    fflush(stderr);

    // ── Step 2: Create btMultiBody (fixed base = kinematic) ──────────────────
    m_mb = new btMultiBody(numLinks, 0.f, btVector3(0,0,0),
                           /*fixedBase*/true, /*canSleep*/false);
    m_mb->setHasSelfCollision(true);
    m_mb->setLinearDamping(0.f);
    m_mb->setAngularDamping(0.01f);

    for(int i = 0; i < numLinks; ++i) {
        auto& info      = m_links[i];
        SceneNode* node = info.node;
        int parentIdx   = info.mbLinkIdx; // stored parent index (-1 or ≥0)

        // Parent node for geometry computation
        SceneNode* parentNode = (parentIdx < 0) ? armBaseNode : m_links[parentIdx].node;

        const float mass = node->inertial.mass;
        // Inertia: MJCF diagInertia is in the body's MJCF-local frame (Z-up).
        // We keep the values as-is since small inaccuracies in inertia tensor
        // orientation have negligible effect under high-kp position servo control.
        const Vec3& Id = node->inertial.diagInertia;
        btVector3 inertia(std::max(1e-6f, Id.x),
                          std::max(1e-6f, Id.y),
                          std::max(1e-6f, Id.z));

        // ── World-space geometric quantities at rest (q=0) ────────────────────
        // Pivot: position of the joint DOF in world space.
        // T_pivot = parent.worldTransform * node.localTransform()
        // (localTransform = T(localPos)*R(localRot), NOT including joint.transform())
        Mat4 Tpivot    = parentNode->worldTransform * node->localTransform();
        Vec3 pivotW    = Tpivot.transformPoint({0,0,0});

        // Joint axis in world space (transforms MJCF-local axis through FK chain)
        Vec3 axisW     = Tpivot.transformDir(node->joint.axis).normalized();

        // Parent and child world rotations
        Quaternion Rp  = worldRot(parentNode);
        Quaternion Rc  = worldRot(node);

        // Parent COM world position
        Vec3 parentComW = parentNode->worldTransform.transformPoint(parentNode->inertial.com);

        // This link COM world position
        Vec3 thisComW   = node->worldTransform.transformPoint(node->inertial.com);

        // ── btMultiBody frame vectors ─────────────────────────────────────────
        // rotParentToThis: rotation mapping parent local frame → child local frame.
        //   v_child = rotParentToThis.rotate(v_parent)
        //   = (Rc^-1 * Rp).rotate(v_parent)
        Quaternion rotPtoC = (Rc.conjugate() * Rp).normalized();

        // parentComToThisPivot in parent local frame
        Vec3 parentComToPivot_local = toLocal(Rp, pivotW - parentComW);

        // thisPivotToThisCom in child local frame
        Vec3 pivotToChildCom_local  = toLocal(Rc, thisComW - pivotW);

        // Joint axis in child local frame
        Vec3 axisInChild = toLocal(Rc, axisW);

        // ── Setup joint ───────────────────────────────────────────────────────
        btQuaternion btRot = toBt(rotPtoC);
        btVector3    btParentPivot(parentComToPivot_local.x,
                                   parentComToPivot_local.y,
                                   parentComToPivot_local.z);
        btVector3    btPivotCom(pivotToChildCom_local.x,
                                pivotToChildCom_local.y,
                                pivotToChildCom_local.z);

        if(info.isRevolute) {
            btVector3 btAxis(axisInChild.x, axisInChild.y, axisInChild.z);
            m_mb->setupRevolute(i, mass, inertia, parentIdx,
                                btRot, btAxis,
                                btParentPivot, btPivotCom,
                                /*disableParentCollision*/true);
        } else {
            // Prismatic (slide) joint
            btVector3 btAxis(axisInChild.x, axisInChild.y, axisInChild.z);
            m_mb->setupPrismatic(i, mass, inertia, parentIdx,
                                 btRot, btAxis,
                                 btParentPivot, btPivotCom,
                                 /*disableParentCollision*/true);
        }

    }

    // ── Finalize ─────────────────────────────────────────────────────────────
    // Must be called BEFORE setJointPos/Vel (allocates internal state buffer).
    m_mb->finalizeMultiDof();

    // Initialize joint positions/velocities AFTER finalizeMultiDof.
    for(int i = 0; i < numLinks; ++i) {
        m_mb->setJointPos(i, m_links[i].node->joint.value);
        m_mb->setJointVel(i, 0.f);
    }

    // Set base world transform from arm_base_link
    setBaseTransform(armBaseNode->worldTransform);

    world->addMultiBody(m_mb);

    // ── Step 4: Build collision shapes and link colliders ─────────────────────
    // Base collider (arm_base_link itself — no collision shape needed since it's
    // inside the robot body and already handled by the drive-base kinematic body)
    m_colliders.emplace_back(nullptr);  // placeholder for base

    for(int i = 0; i < numLinks; ++i) {
        SceneNode* node = m_links[i].node;

        btConvexHullShape* shape = buildShape(node, meshPaths);
        m_shapes.emplace_back(shape);  // takes ownership (may be nullptr)

        auto* col = new btMultiBodyLinkCollider(m_mb, i);
        if(shape) {
            col->setCollisionShape(shape);
            Quaternion rot = worldRot(node);
            Vec3 pos = worldPos(node);
            col->setWorldTransform(btTransform(toBt(rot), toBt(pos)));
            col->setFriction(0.7f);
            col->setRestitution(0.05f);
        }
        m_colliders.emplace_back(col);
        m_mb->getLink(i).m_collider = col;

        // Collision group:
        //   arm links (arm_l/r_link*): ROBOT_LINK, collides with env + objs
        //   hand links (hx5, finger): HAND, can push the can
        const auto& nm = node->name;
        bool isHand = hasPrefix(nm, "hx5_") || hasPrefix(nm, "finger_")
                   || hasPrefix(nm, "thumb_");
        int grp  = isHand ? ColGroup::HAND       : ColGroup::ROBOT_LINK;
        int mask = isHand ? ColGroup::MASK_HAND   : ColGroup::MASK_ROBOT_LINK;
        world->addCollisionObject(col, grp, mask);
    }

    // ── Step 5: Joint limits ──────────────────────────────────────────────────
    for(int i = 0; i < numLinks; ++i) {
        const Joint& j = m_links[i].node->joint;
        if(!j.hasLimits) continue;
        auto* lim = new btMultiBodyJointLimitConstraint(
            m_mb, i, j.limitLo, j.limitHi);
        m_limits.emplace_back(lim);
        world->addMultiBodyConstraint(lim);
    }

    printf("[ArticulatedRobot] Built %d links, %d joint limits\n",
           numLinks, (int)m_limits.size());
}

// ── setBaseTransform ──────────────────────────────────────────────────────────
// Repositions the kinematic btMultiBody base each frame to follow arm_base_link.

void ArticulatedRobot::setBaseTransform(const Mat4& armBaseWorldT) {
    if(!m_mb) return;
    Quaternion rot = armBaseWorldT.extractRotation();
    Vec3       pos = armBaseWorldT.transformPoint({0,0,0});
    btTransform t(toBt(rot), toBt(pos));
    m_mb->setBaseWorldTransform(t);
    // Also update base collider (if any)
    if(!m_colliders.empty() && m_colliders[0])
        m_colliders[0]->setWorldTransform(t);
}

// ── setJointTarget ────────────────────────────────────────────────────────────

void ArticulatedRobot::setJointTarget(SceneNode* node, float q) {
    for(auto& info : m_links) {
        if(info.node == node) {
            info.targetQ = q;
            return;
        }
    }
}

// ── applyServoTorques ─────────────────────────────────────────────────────────
// PD position servo: τ = kp*(target - q) - damping*qdot
// Saturated to [forceMin, forceMax] from MJCF actuator forcerange.
// This exactly matches MuJoCo's <position kp="..."> actuator model.

void ArticulatedRobot::applyServoTorques() {
    if(!m_mb) return;
    for(int i = 0; i < (int)m_links.size(); ++i) {
        const auto& info = m_links[i];

        // Fallback kp for links whose MJCF actuator wasn't parsed (kp==0).
        // Without this, gravity drives joints to limits with no restoring torque.
        float effectiveKp = info.kp;
        if(effectiveKp <= 0.f && info.node) {
            const auto& nm = info.node->name;
            if(hasPrefix(nm, "arm_l_link") || hasPrefix(nm, "arm_r_link"))
                effectiveKp = 300.f;  // typical FFW-SH5 arm joint kp
            else if(hasPrefix(nm, "hx5_") || hasPrefix(nm, "finger_") || hasPrefix(nm, "thumb_"))
                effectiveKp = 30.f;
        }
        if(effectiveKp <= 0.f) continue;

        float q      = m_mb->getJointPos(i);
        float qdot   = m_mb->getJointVel(i);
        float target = info.node ? info.node->joint.value : info.targetQ;
        float tau    = effectiveKp * (target - q) - info.damping * qdot;

        // Use MJCF forcerange if valid; otherwise clamp at ±effectiveKp*10 (practical safety)
        float fMax = (info.forceMax < 1e8f) ? info.forceMax :  effectiveKp * 10.f;
        float fMin = (info.forceMin > -1e8f) ? info.forceMin : -effectiveKp * 10.f;
        if(tau > fMax) tau = fMax;
        if(tau < fMin) tau = fMin;

        m_mb->addJointTorque(i, tau);
    }
}

// ── syncToFK ─────────────────────────────────────────────────────────────────
// After physics.step(), copy Featherstone joint positions back into SceneNode.
// Then call model->update() to refresh worldTransforms for rendering.

void ArticulatedRobot::syncToFK() {
    if(!m_mb) return;
    for(int i = 0; i < (int)m_links.size(); ++i) {
        float q = m_mb->getJointPos(i);
        // Clamp to joint limits to prevent render glitches from physics explosion
        auto* node = m_links[i].node;
        if(node->joint.hasLimits) {
            if(q < node->joint.limitLo) q = node->joint.limitLo;
            if(q > node->joint.limitHi) q = node->joint.limitHi;
        }
        node->joint.value    = q;
        node->joint.valueSag = 0.f;
    }
}

// ── colliderForNode ───────────────────────────────────────────────────────────

btMultiBodyLinkCollider* ArticulatedRobot::colliderForNode(const SceneNode* node) const {
    for(int i = 0; i < (int)m_links.size(); ++i) {
        if(m_links[i].node == node && i+1 < (int)m_colliders.size())
            return m_colliders[i+1].get();
    }
    return nullptr;
}

// ── clear ─────────────────────────────────────────────────────────────────────

void ArticulatedRobot::clear(btMultiBodyDynamicsWorld* world) {
    for(auto& lim : m_limits)
        if(lim) world->removeMultiBodyConstraint(lim.get());
    m_limits.clear();

    for(auto& col : m_colliders)
        if(col) world->removeCollisionObject(col.get());
    m_colliders.clear();
    m_shapes.clear();

    if(m_mb) {
        world->removeMultiBody(m_mb);
        delete m_mb;
        m_mb = nullptr;
    }
    m_links.clear();
    m_world = nullptr;
}
