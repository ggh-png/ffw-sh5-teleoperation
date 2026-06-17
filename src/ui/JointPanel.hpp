#pragma once
#include "robot/RobotModel.hpp"
#include <vector>
#include <string>

// Dear ImGui panel: joint sliders + teleoperation status
class JointPanel {
public:
    int selectedIndex = 0; // currently highlighted joint

    // Draw the panel. Returns true if any joint value was changed.
    bool draw(RobotModel& model);

private:
    // Cached flat list of controllable joints (rebuilt when model changes)
    struct JointEntry {
        std::string name;
        SceneNode*  node = nullptr;
    };
    std::vector<JointEntry> m_joints;
    bool m_dirty = true;

    void rebuildList(RobotModel& model);
};
