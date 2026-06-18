#include "HandPanel.hpp"
#include "robot/ForwardKinematics.hpp"
#include "imgui.h"
#include <cmath>
#include <cstring>

// ── Joint name tables ─────────────────────────────────────────────────────────
// Left thumb:  finger_l_joint 1-4  (CMC, MCPyaw, MCPpitch, IP)
// Left fingers: finger_l_joint 5-8, 9-12, 13-16, 17-20  (4 fingers × 4 joints)
// Right: finger_r_joint (same numbering)

static const char* kThumbJoints[2][4] = {
    {"finger_l_joint1","finger_l_joint2","finger_l_joint3","finger_l_joint4"},
    {"finger_r_joint1","finger_r_joint2","finger_r_joint3","finger_r_joint4"},
};
// 4 fingers × 4 joints (PIP=1, DIP=2, TIP=3 are the closing joints; MCP=0 = spread)
static const char* kFingerJoints[2][4][4] = {
    {   // Left
        {"finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8"},
        {"finger_l_joint9", "finger_l_joint10","finger_l_joint11","finger_l_joint12"},
        {"finger_l_joint13","finger_l_joint14","finger_l_joint15","finger_l_joint16"},
        {"finger_l_joint17","finger_l_joint18","finger_l_joint19","finger_l_joint20"},
    },
    {   // Right
        {"finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8"},
        {"finger_r_joint9", "finger_r_joint10","finger_r_joint11","finger_r_joint12"},
        {"finger_r_joint13","finger_r_joint14","finger_r_joint15","finger_r_joint16"},
        {"finger_r_joint17","finger_r_joint18","finger_r_joint19","finger_r_joint20"},
    },
};

// ── Target angles at full closure (t=1) ──────────────────────────────────────
// Thumb:  [CMC, MCPyaw, MCPpitch, IP]
static const float kThumbClose[4] = {
    0.f,          // CMC: leave neutral (±90° range)
    1.0472f,      // MCP yaw: 60° opposition toward palm
    -1.3f,        // MCP pitch: -74° flex
    -1.3f,        // IP: -74° flex
};
// Finger:  [MCP-spread, PIP, DIP, TIP]  (MCP spread stays neutral)
static const float kFingerClose[4] = {
    0.f,          // MCP spread: keep 0 (neutral)
    1.8f,         // PIP: ~103° flex
    1.4f,         // DIP: ~80° flex
    1.4f,         // TIP: ~80° flex
};

// Helper: interpolate joint value, enforce clamp
static void setJoint(RobotModel& model, const char* name, float target) {
    auto* node = model.findJoint(name);
    if(!node) return;
    node->joint.value = target;
    node->joint.clamp();
}

// ── HandPanel::applyToModel ───────────────────────────────────────────────────
void HandPanel::applyToModel(RobotModel& model,
                              float thumbT_L, float fingerT_L,
                              float thumbT_R, float fingerT_R)
{
    const float tvals[2]  = {thumbT_L,  thumbT_R};
    const float fvals[2]  = {fingerT_L, fingerT_R};

    for(int side = 0; side < 2; ++side) {
        float tt = tvals[side];   // thumb grip 0..1
        float ft = fvals[side];   // finger grip 0..1

        // Left (side=0) and right (side=1) thumbs have MIRRORED joint ranges:
        //   Left  MCPpitch/IP range [-π/2, 0]  → close = negative
        //   Right MCPpitch/IP range [0,  π/2]  → close = positive
        // 핑거 kThumbJoints 2번만 90도 꺾인 상태에서 시작, 나머지는 0에서 시작. (손가락이 펴진 상태)
        const float s = (side == 0) ? 1.f : -1.f;
        setJoint(model, kThumbJoints[side][0], kThumbClose[0] * tt);      // CMC
        setJoint(model, kThumbJoints[side][1], s * 1.5708f);               // MCPyaw: ±90° fixed default
        setJoint(model, kThumbJoints[side][2], s * kThumbClose[2] * tt);  // MCP pitch
        setJoint(model, kThumbJoints[side][3], s * kThumbClose[3] * tt);  // IP
        // 4 fingers — only PIP (joint[1]) has a 90° rest offset; rest start at 0
        for(int f = 0; f < 4; ++f) {
            setJoint(model, kFingerJoints[side][f][0], kFingerClose[0] * ft);
            setJoint(model, kFingerJoints[side][f][1], kFingerClose[1] * ft);
            setJoint(model, kFingerJoints[side][f][2], kFingerClose[2] * ft);
            setJoint(model, kFingerJoints[side][f][3], kFingerClose[3] * ft);
        }
    }
}

// ── HandPanel::gripStrength ───────────────────────────────────────────────────
float HandPanel::gripStrength(int side) const {
    float t = 0.5f * (thumbGrip[side] + fingerGrip[side]);
    return t * torqueLimit[side];  // Nm
}

// ── HandPanel::draw ───────────────────────────────────────────────────────────
bool HandPanel::draw() {
    bool changed = false;
    ImGui::SetNextWindowSize({310, 280}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({10, 430},  ImGuiCond_FirstUseEver);
    ImGui::Begin("Hand Control");

    // Left hand column
    ImGui::BeginGroup();
    ImGui::TextColored({0.3f,0.9f,0.3f,1.f}, "LEFT HAND");
    changed |= drawHandSide(0);
    ImGui::EndGroup();

    ImGui::SameLine(160);

    // Right hand column
    ImGui::BeginGroup();
    ImGui::TextColored({0.9f,0.4f,0.3f,1.f}, "RIGHT HAND");
    changed |= drawHandSide(1);
    ImGui::EndGroup();

    ImGui::Separator();
    ImGui::TextDisabled("Z=L grip  X=R grip  (or sliders above)");
    ImGui::End();
    return changed;
}

bool HandPanel::drawHandSide(int side) {
    bool changed = false;
    char buf[64];

    // Torque limit
    ImGui::SetNextItemWidth(120);
    snprintf(buf, sizeof(buf), "Torque##tq%d", side);
    if(ImGui::SliderFloat(buf, &torqueLimit[side], 0.1f, 2.0f))
        changed = true;

    // Thumb grip
    ImGui::SetNextItemWidth(120);
    snprintf(buf, sizeof(buf), "Thumb##th%d", side);
    ImVec4 thumbCol = {0.6f + 0.4f*thumbGrip[side],
                       0.9f - 0.5f*thumbGrip[side], 0.2f, 1.f};
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, thumbCol);
    if(ImGui::SliderFloat(buf, &thumbGrip[side], 0.f, 1.f))
        changed = true;
    ImGui::PopStyleColor();

    // Finger grip
    ImGui::SetNextItemWidth(120);
    snprintf(buf, sizeof(buf), "Fingers##fg%d", side);
    ImVec4 fingCol = {0.6f + 0.4f*fingerGrip[side],
                      0.9f - 0.5f*fingerGrip[side], 0.2f, 1.f};
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, fingCol);
    if(ImGui::SliderFloat(buf, &fingerGrip[side], 0.f, 1.f))
        changed = true;
    ImGui::PopStyleColor();

    // Quick grip / open buttons
    snprintf(buf, sizeof(buf), "GRIP##gb%d", side);
    if(ImGui::Button(buf, {55,0})) {
        thumbGrip[side] = fingerGrip[side] = 1.f;
        changed = true;
    }
    ImGui::SameLine();
    snprintf(buf, sizeof(buf), "OPEN##ob%d", side);
    if(ImGui::Button(buf, {55,0})) {
        thumbGrip[side] = fingerGrip[side] = 0.f;
        changed = true;
    }

    // Status
    float d = graspDist[side];
    float gs = gripStrength(side);
    if(isGrasping[side])
        ImGui::TextColored({0.2f,1.f,0.4f,1.f}, "GRIPPING %.2fNm", gs);
    else if(graspReady[side])
        ImGui::TextColored({1.f,0.9f,0.1f,1.f}, "IN RANGE %.3fm", d);
    else
        ImGui::TextColored({0.7f,0.7f,0.7f,1.f}, "dist %.3fm", d < 1e8f ? d : 0.f);

    return changed;
}
