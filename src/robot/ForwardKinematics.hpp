#pragma once
#include "SceneNode.hpp"
#include "math/Math.hpp"
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

class ForwardKinematics {
public:
    // Compute world transforms for all nodes starting from root.
    // parentWorld is the transform of the parent coordinate frame
    // (pass Mat4::identity() for the true root).
    static void compute(SceneNode& root,
                        const Mat4& parentWorld = Mat4::identity())
    {
        // 1. Fixed body offset in parent frame
        Mat4 bodyLocal = root.localTransform();

        // 2. Variable joint transform
        Mat4 jointT = root.joint.transform();

        // 3. World transform
        root.worldTransform = parentWorld * bodyLocal * jointT;

        // 4. Recurse into children
        for(auto& child : root.children)
            compute(*child, root.worldTransform);
    }

    // Collect all nodes in depth-first order (useful for Jacobian)
    static void collectNodes(SceneNode& root,
                             std::vector<SceneNode*>& out)
    {
        out.push_back(&root);
        for(auto& child : root.children)
            collectNodes(*child, out);
    }

    // Find node by name (DFS), returns nullptr if not found
    static SceneNode* findByName(SceneNode& root, const std::string& name) {
        if(root.name == name) return &root;
        for(auto& child : root.children) {
            auto* r = findByName(*child, name);
            if(r) return r;
        }
        return nullptr;
    }

    // Get world position of a node's origin
    static Vec3 worldPosition(const SceneNode& node) {
        return node.worldTransform.transformPoint({0,0,0});
    }

    // Walk the kinematic chain from node up to (but not including) root,
    // collecting ancestors. Used to build the Jacobian column set.
    static std::vector<SceneNode*> chain(SceneNode* endEffector,
                                         SceneNode* base)
    {
        std::vector<SceneNode*> chain;
        SceneNode* n = endEffector;
        while(n && n != base) {
            chain.push_back(n);
            n = n->parent;
        }
        return chain; // from ee toward base
    }
};
