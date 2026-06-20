#pragma once
#include "robot/RobotModel.hpp"

// Per-hand finger grip controller.
// Joints are driven by position targets computed from grip values (0=open,1=closed).
// Physics grip force (friction model) is proportional to torqueLimit × gripValue.
class HandPanel {
public:
    // Grip state — index 0=Left, 1=Right
    float thumbGrip[2]   = {0.f, 0.f};   // 0=open 1=fully flexed
    float fingerGrip[2]  = {0.f, 0.f};   // 0=open 1=fully flexed (all 4 fingers)
    float torqueLimit[2] = {0.5f, 0.5f}; // Nm max — scales grip force in physics

    // Set by main loop each frame (physics query results)
    float graspDist[2]   = {1e9f, 1e9f};
    bool  graspReady[2]  = {false, false};
    bool  isGrasping[2]  = {false, false};

    // Set to true by draw() when "Reset Objects" button is pressed.
    // Main loop should act on this and clear it to false.
    bool resetObjects = false;

    // Draw ImGui window.  Returns true if any joint value changed.
    bool draw();

    // Apply current grip values to model joint angles.
    // Call after draw() every frame.
    static void applyToModel(RobotModel& model,
                              float thumbT_L, float fingerT_L,
                              float thumbT_R, float fingerT_R);

    // Grip force for physics (0..1 normalized, scaled inside PhysicsWorld)
    float gripStrength(int side) const;

private:
    bool drawHandSide(int side);
};
