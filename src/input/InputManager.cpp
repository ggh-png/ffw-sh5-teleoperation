#include "InputManager.hpp"
#include <cstring>

InputManager* InputManager::s_instance = nullptr;

void InputManager::scrollCallback(GLFWwindow*, double, double yoff) {
    if(s_instance) s_instance->m_accumScroll += static_cast<float>(yoff);
}

void InputManager::init(GLFWwindow* window) {
    m_window = window;
    s_instance = this;
    glfwSetScrollCallback(window, scrollCallback);
    std::memset(m_prevKeys, 0, sizeof(m_prevKeys));
}

bool InputManager::keyDown(int key) const {
    return glfwGetKey(m_window, key) == GLFW_PRESS;
}

bool InputManager::keyPressed(int key) const {
    return glfwGetKey(m_window, key) == GLFW_PRESS &&
           m_prevKeys[key] == GLFW_RELEASE;
}

void InputManager::update(float dt) {
    // ── Mouse drag ────────────────────────────────────────────────────────
    double cx, cy;
    glfwGetCursorPos(m_window, &cx, &cy);
    m_dx = static_cast<float>(cx - m_lastX);
    m_dy = static_cast<float>(cy - m_lastY);
    m_lastX = cx; m_lastY = cy;

    int lb = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT);
    int mb = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE);
    m_leftDrag = (lb == GLFW_PRESS);
    m_midDrag  = (mb == GLFW_PRESS);

    // ── Scroll ────────────────────────────────────────────────────────────
    m_scrollDY    = m_accumScroll;
    m_accumScroll = 0.f;

    // ── Joint selection ───────────────────────────────────────────────────
    if(m_jointCount > 0) {
        if(keyPressed(GLFW_KEY_TAB))
            m_selectedJoint = (m_selectedJoint + 1) % m_jointCount;
    }

    // ── Teleop command ────────────────────────────────────────────────────
    buildTeleopCmd(dt);

    // ── Edge detection bookkeeping ────────────────────────────────────────
    for(int k = 0; k <= GLFW_KEY_LAST; ++k)
        m_prevKeys[k] = glfwGetKey(m_window, k);
}

void InputManager::buildTeleopCmd(float dt) {
    constexpr float kBaseSpeed  = 0.5f;  // m/s
    constexpr float kLiftSpeed  = 0.3f;
    constexpr float kJointSpeed = 0.8f;  // rad/s

    m_cmd = {};

    // WASD: base XZ movement
    if(keyDown(GLFW_KEY_W)) m_cmd.baseVelocity.z -= kBaseSpeed;
    if(keyDown(GLFW_KEY_S)) m_cmd.baseVelocity.z += kBaseSpeed;
    if(keyDown(GLFW_KEY_A)) m_cmd.baseVelocity.x -= kBaseSpeed;
    if(keyDown(GLFW_KEY_D)) m_cmd.baseVelocity.x += kBaseSpeed;

    // Q/E: lift
    if(keyDown(GLFW_KEY_Q)) m_cmd.liftVelocity += kLiftSpeed;
    if(keyDown(GLFW_KEY_E)) m_cmd.liftVelocity -= kLiftSpeed;

    // Arrow keys: selected joint
    if(keyDown(GLFW_KEY_UP))    m_cmd.selectedJointDelta =  kJointSpeed * dt;
    if(keyDown(GLFW_KEY_DOWN))  m_cmd.selectedJointDelta = -kJointSpeed * dt;
}
