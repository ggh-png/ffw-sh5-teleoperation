#pragma once
#include "robot/RobotModel.hpp"
#include "math/Vec3.hpp"
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

    // IK targets: position (base-local Y-up)
    Vec3       ikTargetL    = {0.3f,  1.2f, -0.4f};
    Vec3       ikTargetR    = {0.3f,  1.2f,  0.4f};

    // Orientation mode: wrist joints driven directly by RPY sliders (FK, not IK).
    // [side][0]=arm_*_joint5(roll), [1]=joint6(pitch), [2]=joint7(yaw)  — radians
    bool       ikUseOrientation = false;
    float      wristRad[2][3]   = {};
    // Set true by draw() when orientation toggle is first enabled,
    // so main.cpp can initialize wristRad from current model joints.
    bool       initWristFromModel = false;

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
