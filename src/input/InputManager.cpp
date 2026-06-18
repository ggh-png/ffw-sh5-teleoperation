#include "InputManager.hpp"
#include <cstring>
#include <cmath>

InputManager* InputManager::s_instance = nullptr;

void InputManager::scrollCallback(GLFWwindow*, double, double yoff) {
    if(s_instance) s_instance->m_accumScroll += static_cast<float>(yoff);
}

void InputManager::init(GLFWwindow* window) {
    m_window = window;
    s_instance = this;
    glfwSetScrollCallback(window, scrollCallback);
    std::memset(m_prevKeys, 0, sizeof(m_prevKeys));
    // Init cursor position so the first frame has zero delta
    glfwGetCursorPos(window, &m_lastX, &m_lastY);
    m_firstFrame = true;
}

bool InputManager::keyDown(int key) const {
    return glfwGetKey(m_window, key) == GLFW_PRESS;
}

bool InputManager::keyPressed(int key) const {
    return glfwGetKey(m_window, key) == GLFW_PRESS &&
           m_prevKeys[key] == GLFW_RELEASE;
}

void InputManager::update(float dt, bool wantCapture) {
    m_captured = wantCapture;

    // ── Cursor tracking (always, so delta is correct when focus returns) ───
    double cx, cy;
    glfwGetCursorPos(m_window, &cx, &cy);

    if(m_firstFrame) {
        // Suppress any delta on the very first update
        m_dx = 0; m_dy = 0;
        m_firstFrame = false;
    } else {
        m_dx = static_cast<float>(cx - m_lastX);
        m_dy = static_cast<float>(cy - m_lastY);
    }
    m_lastX = cx; m_lastY = cy;

    int lb = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT);
    int mb = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE);
    m_leftDrag = (lb == GLFW_PRESS);
    m_midDrag  = (mb == GLFW_PRESS);

    // ── Scroll (suppress output when captured, but still drain accumulator) ─
    m_scrollDY    = m_accumScroll;
    m_accumScroll = 0.f;

    // ── Edge detection bookkeeping ─────────────────────────────────────────
    for(int k = GLFW_KEY_SPACE; k <= GLFW_KEY_LAST; ++k)
        m_prevKeys[k] = glfwGetKey(m_window, k);

    // ── Joint selection + teleop (only when not captured) ─────────────────
    if(wantCapture) {
        m_cmd = {};
        return;
    }

    if(m_jointCount > 0 && keyPressed(GLFW_KEY_TAB))
        m_selectedJoint = (m_selectedJoint + 1) % m_jointCount;

    buildTeleopCmd(dt);
}

void InputManager::buildTeleopCmd(float dt) {
    constexpr float kBaseSpeed  = 0.5f;   // m/s
    constexpr float kYawSpeed   = 1.2f;   // rad/s
    constexpr float kLiftSpeed  = 0.3f;   // m/s
    constexpr float kJointSpeed = 0.8f;   // rad/s

    m_cmd = {};

    // Robot-local movement: +X = robot forward (matches MJCF +X = visual forward)
    // +Z = right strafe, -Z = left strafe
    if(keyDown(GLFW_KEY_W)) m_cmd.baseVelocity.x += kBaseSpeed;
    if(keyDown(GLFW_KEY_S)) m_cmd.baseVelocity.x -= kBaseSpeed;
    if(keyDown(GLFW_KEY_A)) m_cmd.baseVelocity.z -= kBaseSpeed;
    if(keyDown(GLFW_KEY_D)) m_cmd.baseVelocity.z += kBaseSpeed;

    // Left/Right arrows = yaw rotation
    if(keyDown(GLFW_KEY_LEFT))  m_cmd.yawRate =  kYawSpeed;
    if(keyDown(GLFW_KEY_RIGHT)) m_cmd.yawRate = -kYawSpeed;

    if(keyDown(GLFW_KEY_Q)) m_cmd.liftVelocity += kLiftSpeed;
    if(keyDown(GLFW_KEY_E)) m_cmd.liftVelocity -= kLiftSpeed;

    if(keyDown(GLFW_KEY_UP))   m_cmd.selectedJointDelta =  kJointSpeed * dt;
    if(keyDown(GLFW_KEY_DOWN)) m_cmd.selectedJointDelta = -kJointSpeed * dt;
}
