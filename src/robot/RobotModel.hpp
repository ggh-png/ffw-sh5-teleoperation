#pragma once
#include "SceneNode.hpp"
#include "ForwardKinematics.hpp"
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

// Aggregates the robot's scene graph + mesh index mapping.
// Populated by MJCFParser, consumed by Renderer.
struct RobotModel {
    std::unique_ptr<SceneNode> root;        // scene graph root

    // Mesh file paths in load order; SceneNode::meshIndex indexes here
    std::vector<std::string> meshPaths;

    // Joint name → node pointer for quick access
    std::unordered_map<std::string, SceneNode*> jointMap;

    // Build jointMap by walking the tree (call after parsing)
    void buildJointMap() {
        jointMap.clear();
        if(!root) return;
        std::vector<SceneNode*> nodes;
        ForwardKinematics::collectNodes(*root, nodes);
        for(auto* n : nodes) {
            if(!n->joint.name.empty())
                jointMap[n->joint.name] = n;
        }
    }

    void update() {
        if(root) ForwardKinematics::compute(*root);
    }

    SceneNode* findJoint(const std::string& name) {
        auto it = jointMap.find(name);
        return it != jointMap.end() ? it->second : nullptr;
    }

    void setJointValue(const std::string& name, float value) {
        auto* node = findJoint(name);
        if(node) {
            node->joint.value = value;
            node->joint.clamp();
        }
    }

    // Collect all nodes for rendering
    std::vector<SceneNode*> allNodes() {
        std::vector<SceneNode*> out;
        if(root) ForwardKinematics::collectNodes(*root, out);
        return out;
    }
};
