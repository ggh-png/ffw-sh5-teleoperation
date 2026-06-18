#pragma once
#include "math/Vec3.hpp"
#include <GLFW/glfw3.h>
#include <string>

class InputManager {
public:
    struct TeleopCmd {
        Vec3  baseVelocity       = {0,0,0}; // WASD → robot-local XZ (m/s)
        float yawRate            = 0.f;     // ←/→ arrows → yaw (rad/s)
        float liftVelocity       = 0.f;     // Q/E → lift (m/s)
        float selectedJointDelta = 0.f;     // ↑↓ → selected joint (rad)
    };

    void init(GLFWwindow* window);

    // Must be called EVERY frame (even when ImGui has focus) to keep cursor
    // tracking current.  Pass wantCapture=true to suppress teleop/camera output.
    void update(float dt, bool wantCapture = false);

    TeleopCmd teleopCmd() const { return m_cmd; }

    int  selectedJointIndex() const { return m_selectedJoint; }
    void setJointCount(int n)       { m_jointCount = n; }

    bool  leftDragging()  const { return m_leftDrag && !m_captured; }
    bool  middleDragging()const { return m_midDrag  && !m_captured; }
    float mouseDX()       const { return m_captured ? 0.f : m_dx; }
    float mouseDY()       const { return m_captured ? 0.f : m_dy; }
    float scrollDY()      const { return m_captured ? 0.f : m_scrollDY; }

    bool keyDown(int glfwKey) const;
    bool keyPressed(int glfwKey) const;

private:
    GLFWwindow* m_window       = nullptr;
    int         m_jointCount   = 0;
    int         m_selectedJoint= 0;
    bool        m_captured     = false; // ImGui has mouse/keyboard focus

    TeleopCmd m_cmd;

    bool   m_leftDrag = false, m_midDrag = false;
    double m_lastX = 0, m_lastY = 0;
    bool   m_firstFrame = true;  // suppress delta on first update
    float  m_dx = 0, m_dy = 0;
    float  m_scrollDY = 0;

    int m_prevKeys[GLFW_KEY_LAST + 1] = {};

    static void scrollCallback(GLFWwindow*, double, double yoff);
    static InputManager* s_instance;
    float m_accumScroll = 0.f;

    void buildTeleopCmd(float dt);
};
