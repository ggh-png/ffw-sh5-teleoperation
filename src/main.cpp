#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "io/MJCFParser.hpp"
#include "render/Renderer.hpp"
#include "physics/PhysicsWorld.hpp"
#include "input/InputManager.hpp"
#include "ui/JointPanel.hpp"

#include <cstdio>
#include <stdexcept>
#include <memory>
#include <chrono>

static constexpr int  kWidth  = 1280;
static constexpr int  kHeight = 720;
static constexpr char kTitle[]= "FFW-SH5 Teleoperation";

// ── GLFW callbacks ────────────────────────────────────────────────────────────
static Renderer*     g_renderer = nullptr;

static void framebufferSizeCB(GLFWwindow*, int w, int h) {
    if(g_renderer) g_renderer->resize(w, h);
}

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    // Default asset path; override with command-line argument
    const char* mjcfPath = (argc > 1)
        ? argv[1]
        : "assets/ffw_sh5/ffw_sh5.xml";

    // ── GLFW + OpenGL context ─────────────────────────────────────────────
    if(!glfwInit()) {
        std::fprintf(stderr, "GLFW init failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // MSAA

    GLFWwindow* window = glfwCreateWindow(kWidth, kHeight, kTitle, nullptr, nullptr);
    if(!window) {
        std::fprintf(stderr, "GLFW window creation failed\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    if(glewInit() != GLEW_OK) {
        std::fprintf(stderr, "GLEW init failed\n");
        return 1;
    }

    // ── ImGui ─────────────────────────────────────────────────────────────
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 430");

    // ── Subsystems ────────────────────────────────────────────────────────
    Renderer      renderer;
    InputManager  input;
    PhysicsWorld  physics;
    JointPanel    jointPanel;

    g_renderer = &renderer;
    glfwSetFramebufferSizeCallback(window, framebufferSizeCB);

    renderer.init(kWidth, kHeight);
    input.init(window);

    // ── Load robot model ──────────────────────────────────────────────────
    std::unique_ptr<RobotModel> model;
    try {
        std::printf("[main] Loading MJCF: %s\n", mjcfPath);
        model = MJCFParser::parse(mjcfPath);
        std::printf("[main] Loaded %zu meshes, %zu joints\n",
                    model->meshPaths.size(), model->jointMap.size());
    } catch(const std::exception& e) {
        std::fprintf(stderr, "[main] Model load failed: %s\n"
                             "       Run with path to ffw_sh5.xml as argument.\n",
                     e.what());
        // Continue with empty model for testing the renderer
        model = std::make_unique<RobotModel>();
    }

    input.setJointCount(static_cast<int>(model->jointMap.size()));

    // Collect ordered joint list for keyboard control
    std::vector<SceneNode*> controllableJoints;
    for(auto* n : model->allNodes())
        if(n->joint.hasDOF()) controllableJoints.push_back(n);

    // ── Timing ────────────────────────────────────────────────────────────
    using Clock = std::chrono::steady_clock;
    auto lastTime = Clock::now();

    // ── Main loop ─────────────────────────────────────────────────────────
    while(!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto now = Clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        dt = std::min(dt, 0.05f); // cap at 50ms to avoid spiral of death
        lastTime = now;

        // Don't process input when ImGui has focus
        bool wantCapture = io.WantCaptureKeyboard || io.WantCaptureMouse;

        if(!wantCapture) {
            input.update(dt);

            // Camera orbit
            if(input.leftDragging())
                renderer.camera.onMouseDrag(input.mouseDX(), input.mouseDY());
            if(input.middleDragging())
                renderer.camera.onPan(input.mouseDX(), input.mouseDY());
            renderer.camera.onScroll(input.scrollDY());

            // Teleop → physics base velocity
            auto cmd = input.teleopCmd();
            physics.setBaseVelocity(cmd.baseVelocity);

            // Lift joint
            model->setJointValue("lift_joint", [&]{
                auto* n = model->findJoint("lift_joint");
                if(!n) return 0.f;
                float v = n->joint.value + cmd.liftVelocity * dt;
                return v;
            }());

            // Selected joint keyboard control
            int sel = input.selectedJointIndex();
            jointPanel.selectedIndex = sel;
            if(sel < (int)controllableJoints.size()) {
                auto& j = controllableJoints[sel]->joint;
                j.value += cmd.selectedJointDelta;
                j.clamp();
            }

            // Escape to close
            if(input.keyDown(GLFW_KEY_ESCAPE))
                glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        // Physics step
        physics.step(dt);

        // Feed base position back into robot root
        Vec3 basePos = physics.basePosition();
        if(model->root && !model->root->children.empty()) {
            model->root->children[0]->localPos = basePos;
        }

        // FK update
        model->update();

        // ── Render ────────────────────────────────────────────────────────
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Draw joint panel — applies slider changes directly to model
        if(jointPanel.draw(*model))
            model->update();

        // FPS overlay
        ImGui::SetNextWindowPos({kWidth - 120.f, 10.f}, ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.5f);
        ImGui::Begin("##fps", nullptr,
                     ImGuiWindowFlags_NoDecoration |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoNav);
        ImGui::Text("%.0f FPS", io.Framerate);
        ImGui::Text("dt %.2f ms", dt * 1000.f);
        ImGui::End();

        renderer.render(*model);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ── Cleanup ───────────────────────────────────────────────────────────
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    renderer.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
