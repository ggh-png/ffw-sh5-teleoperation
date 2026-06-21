#include "JointPanel.hpp"
#include "imgui.h"
#include "math/Math.hpp"
#include "robot/Joint.hpp"

// ── Group name string helpers ─────────────────────────────────────────────────

static const char* groupLabel(JointGroup g) {
    switch(g) {
    case JointGroup::All:     return "ALL";
    case JointGroup::LeftArm: return "L-ARM";
    case JointGroup::RightArm:return "R-ARM";
    case JointGroup::Head:    return "HEAD";
    case JointGroup::Lift:    return "LIFT";
    case JointGroup::Wheel:   return "WHEEL";
    }
    return "?";
}

bool JointPanel::matchesGroup(const std::string& name, JointGroup g) {
    auto startsWith = [&](const char* prefix) {
        return name.rfind(prefix, 0) == 0;
    };
    switch(g) {
    case JointGroup::All:      return true;
    case JointGroup::LeftArm:  return startsWith("arm_l") || startsWith("thumb_l") ||
                                       startsWith("finger_l");
    case JointGroup::RightArm: return startsWith("arm_r") || startsWith("thumb_r") ||
                                       startsWith("finger_r");
    case JointGroup::Head:     return startsWith("head");
    case JointGroup::Lift:     return startsWith("lift");
    case JointGroup::Wheel:    return startsWith("left_wheel") ||
                                       startsWith("right_wheel") ||
                                       startsWith("rear_wheel");
    }
    return true;
}

void JointPanel::rebuildList(RobotModel& model) {
    m_joints.clear();
    for(auto* n : model.allNodes()) {
        if(!n->joint.hasDOF()) continue;
        // Free / Ball joints can't be controlled by a scalar slider
        if(n->joint.type == JointType::Free || n->joint.type == JointType::Ball) continue;
        if(!matchesGroup(n->joint.name, group)) continue;
        m_joints.push_back({n->joint.name, n});
    }
    // Clamp selection
    if(selectedIndex >= (int)m_joints.size())
        selectedIndex = m_joints.empty() ? 0 : (int)m_joints.size() - 1;
    m_lastGroup = group;
    m_dirty = false;
}

// ── Main draw ─────────────────────────────────────────────────────────────────

bool JointPanel::draw(RobotModel& model) {
    if(m_dirty || group != m_lastGroup) rebuildList(model);

    // Always clamp after potential rebuild
    if(m_joints.empty())
        selectedIndex = 0;
    else
        selectedIndex = std::max(0, std::min(selectedIndex,
                                             (int)m_joints.size() - 1));

    bool changed = false;

    ImGui::SetNextWindowSize({360, 620}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({10, 10},   ImGuiCond_FirstUseEver);
    ImGui::Begin("Joint Control");

    // ── Group selector ────────────────────────────────────────────────────
    ImGui::Text("Group:");
    ImGui::SameLine();
    JointGroup groups[] = {JointGroup::All, JointGroup::LeftArm,
                           JointGroup::RightArm, JointGroup::Head,
                           JointGroup::Lift, JointGroup::Wheel};
    for(auto g : groups) {
        bool sel = (group == g);
        if(sel) ImGui::PushStyleColor(ImGuiCol_Button, {0.2f,0.5f,0.9f,1.f});
        if(ImGui::SmallButton(groupLabel(g))) { group = g; m_dirty = true; }
        if(sel) ImGui::PopStyleColor();
        ImGui::SameLine();
    }
    ImGui::NewLine();
    ImGui::Separator();

    // ── Mode: FK / IK ─────────────────────────────────────────────────────
    {
        bool fkSel = !ikMode, ikSel = ikMode;
        if(fkSel) ImGui::PushStyleColor(ImGuiCol_Button, {0.2f,0.6f,0.3f,1.f});
        if(ImGui::SmallButton("FK")) ikMode = false;
        if(fkSel) ImGui::PopStyleColor();
        ImGui::SameLine();
        if(ikSel) ImGui::PushStyleColor(ImGuiCol_Button, {0.8f,0.4f,0.1f,1.f});
        if(ImGui::SmallButton("IK")) ikMode = true;
        if(ikSel) ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::Checkbox("Gizmos[G]", &showGizmos);
    }
    ImGui::Separator();

    // ── IK target sliders ─────────────────────────────────────────────────
    if(ikMode) {
        ImGui::Text("IK Targets (base-local Y-up, metres)");

        // Progress bar showing normalized position in range, color-coded by proximity to limits
        auto showPBar = [](float val, float lo, float hi) {
            float frac = (val - lo) / (hi - lo);
            frac = frac < 0.f ? 0.f : (frac > 1.f ? 1.f : frac);
            float margin = frac < 0.5f ? frac : 1.f - frac;  // 0 at limits, 0.5 at center
            ImVec4 col = (margin < 0.1f) ? ImVec4{0.85f,0.18f,0.18f,1.f}
                       : (margin < 0.2f) ? ImVec4{0.9f, 0.65f,0.10f,1.f}
                                         : ImVec4{0.18f,0.65f,0.18f,1.f};
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
            ImGui::ProgressBar(frac, {-1, 5});
            ImGui::PopStyleColor();
        };

        // Left arm position
        ImGui::PushStyleColor(ImGuiCol_FrameBg, {0.10f,0.30f,0.10f,0.8f});
        ImGui::DragFloat("L-X (fwd)##iktlx", &ikTargetL.x, 0.001f,-1.5f,1.5f,"%.3f m");
        showPBar(ikTargetL.x, -1.5f, 1.5f);
        ImGui::DragFloat("L-Y (up) ##iktly", &ikTargetL.y, 0.001f,-0.5f,2.0f,"%.3f m");
        showPBar(ikTargetL.y, -0.5f, 2.0f);
        ImGui::DragFloat("L-Z (lat)##iktlz", &ikTargetL.z, 0.001f,-1.5f,1.5f,"%.3f m");
        showPBar(ikTargetL.z, -1.5f, 1.5f);

        // Left wrist FK (joint5=roll/Z, joint6=pitch/Y, joint7=yaw/X)
        if(ikUseOrientation) {
            float rd = toDeg(wristRad[0][0]);
            float pd = toDeg(wristRad[0][1]);
            float yd = toDeg(wristRad[0][2]);
            bool ch = ImGui::SliderFloat("L-Roll##wrl",  &rd, -180.f, 180.f, "%.1f°");
            ch |=     ImGui::SliderFloat("L-Pitch##wpl", &pd,  -90.f,  90.f, "%.1f°");
            ch |=     ImGui::SliderFloat("L-Yaw##wyl",   &yd, -104.f, 104.f, "%.1f°");
            if(ch) {
                wristRad[0][0] = toRad(rd);
                wristRad[0][1] = toRad(pd);
                wristRad[0][2] = toRad(yd);
                changed = true;
            }
        }
        ImGui::PopStyleColor();

        // Right arm position
        ImGui::PushStyleColor(ImGuiCol_FrameBg, {0.30f,0.10f,0.10f,0.8f});
        ImGui::DragFloat("R-X (fwd)##iktrx", &ikTargetR.x, 0.001f,-1.5f,1.5f,"%.3f m");
        showPBar(ikTargetR.x, -1.5f, 1.5f);
        ImGui::DragFloat("R-Y (up) ##iktry", &ikTargetR.y, 0.001f,-0.5f,2.0f,"%.3f m");
        showPBar(ikTargetR.y, -0.5f, 2.0f);
        ImGui::DragFloat("R-Z (lat)##iktrz", &ikTargetR.z, 0.001f,-1.5f,1.5f,"%.3f m");
        showPBar(ikTargetR.z, -1.5f, 1.5f);

        // Right wrist FK (joint5=roll/Z, joint6=pitch/Y, joint7=yaw/X)
        if(ikUseOrientation) {
            float rd = toDeg(wristRad[1][0]);
            float pd = toDeg(wristRad[1][1]);
            float yd = toDeg(wristRad[1][2]);
            bool ch = ImGui::SliderFloat("R-Roll##wrr",  &rd, -180.f, 180.f, "%.1f°");
            ch |=     ImGui::SliderFloat("R-Pitch##wpr", &pd,  -90.f,  90.f, "%.1f°");
            ch |=     ImGui::SliderFloat("R-Yaw##wyr",   &yd, -104.f, 104.f, "%.1f°");
            if(ch) {
                wristRad[1][0] = toRad(rd);
                wristRad[1][1] = toRad(pd);
                wristRad[1][2] = toRad(yd);
                changed = true;
            }
        }
        ImGui::PopStyleColor();

        bool prevOri = ikUseOrientation;
        ImGui::Checkbox("Wrist FK (Roll/Pitch/Yaw)", &ikUseOrientation);
        if(!prevOri && ikUseOrientation)
            initWristFromModel = true;  // main.cpp reads current joint angles
        ImGui::Separator();
    }

    // ── Nav hint ─────────────────────────────────────────────────────────
    ImGui::Text("TAB=next  UP/DN=adjust  WASD=move  LR=yaw  Q/E=lift");
    ImGui::Separator();

    // ── Selected joint info ───────────────────────────────────────────────
    if(!m_joints.empty() && selectedIndex < (int)m_joints.size()) {
        const SceneNode* sel = m_joints[selectedIndex].node;
        Vec3 wp = sel->worldTransform.transformPoint({0,0,0});
        ImGui::Text("[%d/%d] %s", selectedIndex+1,
                    (int)m_joints.size(),
                    m_joints[selectedIndex].name.c_str());
        ImGui::Text("world  %.3f  %.3f  %.3f", wp.x, wp.y, wp.z);
        ImGui::Text("angle  %.3f rad / %.1f deg",
                    sel->joint.value, toDeg(sel->joint.value));
    }
    ImGui::Separator();

    // ── Joint sliders ─────────────────────────────────────────────────────
    ImGui::Text("Joints: %d", (int)m_joints.size());
    ImGui::BeginChild("JointList", {0, 0}, false);

    for(int i = 0; i < (int)m_joints.size(); ++i) {
        auto& e  = m_joints[i];
        Joint& j = e.node->joint;

        bool selected = (i == selectedIndex);
        if(selected) ImGui::PushStyleColor(ImGuiCol_FrameBg,
                                           {0.2f,0.4f,0.7f,0.6f});

        bool inDeg     = (j.type == JointType::Revolute);
        float dispVal  = inDeg ? toDeg(j.value) : j.value;
        float dispLo   = j.hasLimits ? (inDeg ? toDeg(j.limitLo) : j.limitLo) : -180.f;
        float dispHi   = j.hasLimits ? (inDeg ? toDeg(j.limitHi) : j.limitHi) :  180.f;

        // Out-of-limit highlight
        bool atLimit = j.hasLimits &&
                       (j.value <= j.limitLo + 1e-3f ||
                        j.value >= j.limitHi - 1e-3f);
        if(atLimit) ImGui::PushStyleColor(ImGuiCol_SliderGrab, {0.9f,0.2f,0.2f,1.f});

        ImGui::TextUnformatted(e.name.c_str());
        ImGui::SameLine(150);
        ImGui::SetNextItemWidth(170);
        std::string label = "##j" + std::to_string(i);
        if(ImGui::SliderFloat(label.c_str(), &dispVal, dispLo, dispHi)) {
            j.value = inDeg ? toRad(dispVal) : dispVal;
            j.clamp();
            changed = true;
        }

        if(atLimit) ImGui::PopStyleColor();
        if(selected) ImGui::PopStyleColor();
    }

    ImGui::EndChild();
    ImGui::End();
    return changed;
}
