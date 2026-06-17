#pragma once
#include "math/Vec3.hpp"
#include <GLFW/glfw3.h>
#include <functional>
#include <string>

// Central input handler.
// Provides processed state (not raw events) to the rest of the app.
class InputManager {
public:
    // Teleop command, updated each frame
    struct TeleopCmd {
        Vec3  baseVelocity  = {0,0,0}; // WASD → XZ plane movement (m/s)
        float liftVelocity  = 0.f;     // Q/E → vertical (m/s)
        float selectedJointDelta = 0.f;// arrow keys → selected joint (rad/s)
    };

    void init(GLFWwindow* window);

    // Call once per frame before reading state
    void update(float dt);

    TeleopCmd teleopCmd() const { return m_cmd; }

    // Joint selection (Tab cycles, number keys select)
    int  selectedJointIndex() const { return m_selectedJoint; }
    void setJointCount(int n)       { m_jointCount = n; }

    // Camera drag state (read by Camera)
    bool  leftDragging()  const { return m_leftDrag; }
    bool  middleDragging()const { return m_midDrag;  }
    float mouseDX()       const { return m_dx; }
    float mouseDY()       const { return m_dy; }
    float scrollDY()      const { return m_scrollDY; }

    // Keys
    bool keyDown(int glfwKey) const;
    bool keyPressed(int glfwKey) const; // only true for one frame

private:
    GLFWwindow* m_window    = nullptr;
    int         m_jointCount = 0;
    int         m_selectedJoint = 0;
    TeleopCmd   m_cmd;

    // Mouse state
    bool  m_leftDrag = false, m_midDrag = false;
    double m_lastX = 0, m_lastY = 0;
    float  m_dx = 0, m_dy = 0;
    float  m_scrollDY = 0;

    // Key edge detection
    int m_prevKeys[GLFW_KEY_LAST + 1] = {};

    static void scrollCallback(GLFWwindow*, double, double yoff);
    static InputManager* s_instance;
    float m_accumScroll = 0.f;

    void buildTeleopCmd(float dt);
};
