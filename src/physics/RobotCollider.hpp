#pragma once
#include "math/Math.hpp"
#include "robot/SceneNode.hpp"
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <memory>
#include <vector>
#include <string>

// Builds a kinematic Bullet collision body for every mesh-bearing SceneNode in
// the robot model. Call build() once after the model is loaded, then update()
// each frame after FK (model->update()) to keep all body poses in sync.
class RobotCollider {
public:
    // meshPaths: RobotModel::meshPaths — absolute STL paths, indexed by SceneNode::meshIndex
    void build(btMultiBodyDynamicsWorld* world,
               const std::vector<SceneNode*>& nodes,
               const std::vector<std::string>& meshPaths);

    // Sync every link body's world transform from SceneNode::worldTransform.
    void update();

    // Remove all bodies from the world and free resources.
    void clear(btMultiBodyDynamicsWorld* world);

    int linkCount() const { return (int)m_links.size(); }

    // ── Penetration detection ─────────────────────────────────────────────
    // Used each frame after update() to detect arm/wheel links inside static env bodies.
    // Uses Bullet contactPairTest (bypasses broadphase, tests current transforms directly).
    struct StaticPenetration {
        SceneNode*   node;          // which robot link is penetrating
        Vec3         pushNormal;    // world-space direction to push link OUT of env body
        float        depth;         // penetration depth (m, positive)
        Vec3         contactPoint;  // point on robot link surface (world space)
    };
    std::vector<StaticPenetration> findStaticPenetrations(
        btMultiBodyDynamicsWorld* world,
        const std::vector<btRigidBody*>& envBodies) const;

    // Map a btCollisionObject back to its SceneNode (nullptr if not a robot link).
    SceneNode* nodeForBody(const btCollisionObject* body) const;

    // Returns all rigid bodies whose node name starts with the given prefix.
    // Used to build self-collision env sets (e.g. left arm bodies for right arm checking).
    std::vector<btRigidBody*> bodiesForPrefix(const std::string& prefix) const;

    // Returns the rigid body for the link whose node name exactly matches name, or nullptr.
    btRigidBody* bodyForName(const std::string& name) const;

private:
    struct LinkBody {
        SceneNode*                             node = nullptr;
        std::unique_ptr<btConvexHullShape>     shape;
        std::unique_ptr<btDefaultMotionState>  state;
        std::unique_ptr<btRigidBody>           body;
    };
    std::vector<LinkBody> m_links;

    static btTransform fromMat4(const Mat4& m);
};
