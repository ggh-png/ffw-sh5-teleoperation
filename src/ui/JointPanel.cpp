#include "JointPanel.hpp"
#include "imgui.h"
#include "math/Math.hpp"

void JointPanel::rebuildList(RobotModel& model) {
    m_joints.clear();
    auto nodes = model.allNodes();
    for(auto* n : nodes) {
        if(n->joint.hasDOF())
            m_joints.push_back({n->joint.name, n});
    }
    m_dirty = false;
}

bool JointPanel::draw(RobotModel& model) {
    if(m_dirty) rebuildList(model);

    bool changed = false;

    ImGui::SetNextWindowSize({340, 600}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({10, 10},   ImGuiCond_FirstUseEver);

    ImGui::Begin("Joint Control");

    ImGui::Text("Joints: %d", static_cast<int>(m_joints.size()));
    ImGui::Text("Selected: %s",
        selectedIndex < (int)m_joints.size()
            ? m_joints[selectedIndex].name.c_str() : "—");
    ImGui::Separator();

    ImGui::Text("Nav: TAB=next  ↑↓=adjust");
    ImGui::Text("Base: WASD=move  Q/E=lift");
    ImGui::Separator();

    ImGui::BeginChild("JointList", {0, 0}, false);
    for(int i = 0; i < (int)m_joints.size(); ++i) {
        auto& e = m_joints[i];
        Joint& j = e.node->joint;

        bool selected = (i == selectedIndex);
        if(selected) ImGui::PushStyleColor(ImGuiCol_FrameBg, {0.2f,0.4f,0.7f,0.6f});

        float lo = j.limitLo, hi = j.limitHi;
        float deg = toDeg(j.value);
        float degLo = toDeg(lo), degHi = toDeg(hi);

        bool inDeg = (j.type == JointType::Revolute);
        float displayVal  = inDeg ? deg    : j.value;
        float displayLo   = inDeg ? degLo  : lo;
        float displayHi   = inDeg ? degHi  : hi;

        std::string label = "##" + e.name;
        ImGui::TextUnformatted(e.name.c_str());
        ImGui::SameLine(140);
        ImGui::SetNextItemWidth(160);
        if(ImGui::SliderFloat(label.c_str(), &displayVal, displayLo, displayHi)) {
            j.value = inDeg ? toRad(displayVal) : displayVal;
            j.clamp();
            changed = true;
        }

        if(selected) ImGui::PopStyleColor();
    }
    ImGui::EndChild();

    ImGui::End();
    return changed;
}
