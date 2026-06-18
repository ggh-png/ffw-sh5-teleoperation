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
#include <map>

// ── ArticulatedRobot ─────────────────────────────────────────────────────────
// Builds a Bullet btMultiBody (Featherstone reduced-coordinate articulated body)
// directly from the parsed MJCF SceneNode tree and STL mesh files.
//
// Unlike the previous RobotCollider (kinematic convex hulls + manual Jacobian
// correction), this class makes the robot arm a TRUE PHYSICS BODY:
//   • Mass, inertia and COM from MJCF <inertial>
//   • Collision shapes built from STL meshes (same data as rendering)
//   • Parent-collision disabled per btMultiBody convention (handles <exclude> pairs)
//   • PD servo torques per joint (kp, damping, forcerange from MJCF)
//   • Contact with environment / can handled automatically by Featherstone solver
//   • Self-collision between arm links computed automatically
//
// Usage per frame:
//   1. setBaseTransform(arm_base_link_worldT)   // follow mobile base + lift
//   2. setJointTarget(name, q)                  // from teleop input
//   3. applyServoTorques()                       // before physics.step()
//   4. physics.step(dt)
//   5. syncToFK()                               // arm joints → SceneNode values
//   6. model->update()                          // re-run FK for rendering
class ArticulatedRobot {
public:
    // Build the btMultiBody starting from armBaseNode's children.
    // armBaseNode (arm_base_link) becomes the kinematic base — it is repositioned
    // each frame by setBaseTransform() to follow the mobile base.
    // Only links with inertial.valid==true and inertial.mass>0 are included.
    // meshPaths: RobotModel::meshPaths indexed by SceneNode::meshIndex.
    void build(btMultiBodyDynamicsWorld* world,
               SceneNode* armBaseNode,
               const std::vector<std::string>& meshPaths);

    // Update the btMultiBody base position from the current FK of arm_base_link.
    // Call every frame before applyServoTorques().
    void setBaseTransform(const Mat4& armBaseWorldTransform);

    // Set the target position for a joint. The target is held between frames
    // (i.e. the arm holds position unless explicitly changed).
    void setJointTarget(SceneNode* node, float q);

    // Apply PD servo torques to all joints. Must be called before physics.step().
    void applyServoTorques();

    // Read joint positions back from physics into SceneNode::joint::value.
    // Call after physics.step(), then call model->update() for rendering.
    void syncToFK();

    // Remove all bodies and shapes from the world and free memory.
    void clear(btMultiBodyDynamicsWorld* world);

    bool isBuilt()   const { return m_mb != nullptr; }
    int  linkCount() const { return m_mb ? m_mb->getNumLinks() : 0; }

    // Expose the btMultiBodyLinkCollider for a specific node (nullptr if not found).
    btMultiBodyLinkCollider* colliderForNode(const SceneNode* node) const;

private:
    struct LinkInfo {
        SceneNode*   node;
        int          mbLinkIdx;   // index in btMultiBody (-1 = base, not used here)
        float        kp;
        float        damping;
        float        forceMin;
        float        forceMax;
        float        targetQ     = 0.f;
        bool         isRevolute  = true;
    };

    btMultiBody*                                           m_mb      = nullptr;
    btMultiBodyDynamicsWorld*                              m_world   = nullptr;
    std::vector<LinkInfo>                                  m_links;
    std::vector<std::unique_ptr<btMultiBodyLinkCollider>>  m_colliders; // [0]=base, [i+1]=link i
    std::vector<std::unique_ptr<btConvexHullShape>>        m_shapes;
    std::vector<std::unique_ptr<btMultiBodyJointLimitConstraint>> m_limits;

    // Helper: should this node be included in the btMultiBody?
    static bool shouldInclude(const SceneNode* node);

    // Helper: build btConvexHullShape from STL mesh at meshPaths[node->meshIndex].
    // Returns nullptr if no mesh or mesh is empty.
    static btConvexHullShape* buildShape(const SceneNode* node,
                                          const std::vector<std::string>& meshPaths);

    // Helper: add a btMultiBodyLinkCollider for link linkIdx (-1 = base).
    void addCollider(btConvexHullShape* shape, int linkIdx, const Mat4& worldT);
};
