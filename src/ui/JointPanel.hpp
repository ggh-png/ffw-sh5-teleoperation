#pragma once
#include "robot/RobotModel.hpp"
#include "math/Vec3.hpp"
#include "math/Quaternion.hpp"
#include <vector>
#include <string>

// Joint group filter — determines which joints are shown / TAB-cycled
enum class JointGroup { All, LeftArm, RightArm, Head, Lift, Wheel };

class JointPanel {
public:
    int        selectedIndex = 0;
    bool       showGizmos   = false;

    // Arm control mode
    JointGroup group   = JointGroup::All;
    bool       ikMode  = false;     // false=FK (joint angle), true=IK (ee target)

    // IK targets: position (base-local Y-up) + orientation
    Vec3       ikTargetL    = {0.3f,  1.2f, -0.4f};
    Vec3       ikTargetR    = {0.3f,  1.2f,  0.4f};
    Quaternion ikTargetRotL = Quaternion::identity();
    Quaternion ikTargetRotR = Quaternion::identity();
    bool       ikUseOrientation = false;

    // Set true by draw() when RPY slider was actively dragged this frame.
    // main.cpp uses this to freeze XYZ IK target while orientation is adjusted.
    bool       ikRotChangedL = false;
    bool       ikRotChangedR = false;

    // Returns true if any joint value changed via slider
    bool draw(RobotModel& model);

    // Flat filtered joint list (rebuilt on group change)
    struct JointEntry { std::string name; SceneNode* node = nullptr; };
    const std::vector<JointEntry>& joints() const { return m_joints; }

private:
    std::vector<JointEntry> m_joints;
    JointGroup m_lastGroup = JointGroup::All;
    bool       m_dirty     = true;

    void rebuildList(RobotModel& model);
    static bool matchesGroup(const std::string& name, JointGroup g);
};
