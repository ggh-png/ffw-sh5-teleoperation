#include <epoxy/gl.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "io/MJCFParser.hpp"
#include "render/Renderer.hpp"
#include "physics/PhysicsWorld.hpp"
#include "input/InputManager.hpp"
#include "ui/JointPanel.hpp"
#include "ui/HandPanel.hpp"
#include "robot/InverseKinematics.hpp"
#include "robot/ForwardKinematics.hpp"
#include "physics/RobotCollider.hpp"
#include "physics/HandPhysics.hpp"
#include "io/SceneLoader.hpp"

#include <cstdio>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <cmath>

static constexpr int  kWidth  = 1280;
static constexpr int  kHeight = 720;
static constexpr char kTitle[]= "FFW-SH5 Teleoperation";

// Table geometry constants (shared between physics and rendering)
// Center at Y=0.325, half-ext Y=0.325 → bottom=0 (floor), top=0.65m
static const Vec3  kTablePos     = {0.f, 0.325f, -0.65f};  // closer so arm can reach
static const Vec3  kTableHalfExt = {0.30f, 0.325f, 0.25f};
static const float kTableTopY    = kTablePos.y + kTableHalfExt.y;  // 0.65 m
static const float kCanHalfH     = 0.06f;
static const float kCanRadius    = 0.033f;

static Renderer* g_renderer = nullptr;
static void framebufferSizeCB(GLFWwindow*, int w, int h) {
    if(g_renderer) g_renderer->resize(w, h);
}

int main(int argc, char** argv) {
    const char* mjcfPath = (argc > 1)
        ? argv[1]
        : "assets/ffw_sh5/robotis_ffw/ffw_sh5.xml";

    // ── GLFW + OpenGL ─────────────────────────────────────────────────────
    if(!glfwInit()) { std::fprintf(stderr, "GLFW init failed\n"); return 1; }
    glfwSetErrorCallback([](int c, const char* m){
        std::fprintf(stderr, "GLFW %d: %s\n", c, m);
    });
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(kWidth, kHeight, kTitle, nullptr, nullptr);
    if(!window) { std::fprintf(stderr, "Window failed\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    std::printf("[main] OpenGL %s | %s\n",
                glGetString(GL_VERSION), glGetString(GL_RENDERER));

    // ── ImGui ─────────────────────────────────────────────────────────────
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 430");

    // ── Load model (early, so PhysicsOptions is available before physics init) ──
    std::unique_ptr<RobotModel> model;
    PhysicsOptions physOpts; // populated from MJCF <option>
    try {
        std::printf("[main] Loading MJCF: %s\n", mjcfPath);
        auto parsed = MJCFParser::parse(mjcfPath);
        model    = std::move(parsed.model);
        physOpts = parsed.physics;
        std::printf("[main] Loaded %zu meshes, %zu joints | gravity=%.2f timestep=%.4fs\n",
                    model->meshPaths.size(), model->jointMap.size(),
                    physOpts.gravity, physOpts.timestep);
    } catch(const std::exception& e) {
        std::fprintf(stderr, "[main] Model load failed: %s\n", e.what());
        model = std::make_unique<RobotModel>();
    }
    model->update();

    // ── Derive wheel geometry from MJCF instead of hardcoding ────────────
    // kWheelGroundOfs: height of wheel contact point above robot origin
    //   = sum of MJCF Z offsets (body localPos.z) from world root to wheel_drive
    //     minus wheel radius.
    // MJCF uses Z-up; localPos.z maps to Y in our Y-up physics world.
    float kWheelRadius    = 0.09f;   // fallback (MJCF size="0.09 ...")
    float kWheelGroundOfs = 0.1465f; // fallback
    {
        SceneNode* wdNode = ForwardKinematics::findByName(*model->root, "left_wheel_drive");
        if(wdNode && wdNode->geomRadius > 0.f) {
            kWheelRadius = wdNode->geomRadius;
            float sumZ = 0.f;
            for(SceneNode* n = wdNode; n && n->name != "__world__"; n = n->parent)
                sumZ += n->localPos.z; // MJCF Z = physics Y (after root -90° X rotation)
            kWheelGroundOfs = sumZ - kWheelRadius;
            std::printf("[main] kWheelRadius=%.4fm  kWheelGroundOfs=%.4fm  (derived from MJCF)\n",
                        kWheelRadius, kWheelGroundOfs);
        } else {
            std::printf("[main] kWheelRadius=%.4fm  kWheelGroundOfs=%.4fm  (fallback)\n",
                        kWheelRadius, kWheelGroundOfs);
        }
    }

    // ── Subsystems ────────────────────────────────────────────────────────
    Renderer     renderer;
    InputManager input;
    PhysicsWorld physics(physOpts);
    JointPanel   jointPanel;
    HandPanel    handPanel;

    g_renderer = &renderer;
    glfwSetFramebufferSizeCallback(window, framebufferSizeCB);
    renderer.init(kWidth, kHeight);
    input.init(window);

    // ── Scene loading (replaces hardcoded table + can) ───────────────────
    // Derive scene.xml path from mjcfPath (same directory, different filename)
    {
        std::string sceneDir = mjcfPath;
        auto slash = sceneDir.find_last_of("/\\");
        if(slash != std::string::npos) sceneDir.resize(slash + 1);
        else sceneDir = "./";
        std::string scenePath = sceneDir + "scene.xml";

        auto descs = SceneLoader::load(scenePath);
        physics.spawnObjects(descs);

        // Push render objects: static boxes → renderer.boxes, dynamic → renderer.objects
        for(const auto& d : descs) {
            if(d.type == ObjectDesc::Type::Box)
                renderer.boxes.push_back({d.pos, Quaternion::identity(), d.color, d.halfExtents});
            // Cylinders are driven by physics.objectStates() each frame (done below)
        }
    }

    // Floor visual (always present, physics plane handles collision)
    renderer.boxes.push_back({{0,-.01f,0}, Quaternion::identity(),
                               {0.60f,0.60f,0.60f}, {6,.01f,6}});


    // ── Robot collision (kinematic ConvexHull per link, FK-driven) ──────────────
    RobotCollider robotCollider;
    robotCollider.build(physics.bulletWorld(), model->allNodes(), model->meshPaths);

    // ── Hand physics (btMultiBody finger dynamics for real grasping) ─────────
    HandPhysics handPhysics;
    {
        SceneNode* palmL = ForwardKinematics::findByName(*model->root, "hx5_l_base");
        SceneNode* palmR = ForwardKinematics::findByName(*model->root, "hx5_r_base");
        if(palmL || palmR) {
            handPhysics.build(physics.bulletWorld(), palmL, palmR, model->meshPaths);
        } else {
            std::fprintf(stderr, "[main] WARNING: palm nodes not found — falling back to kinematic grasp\n");
        }
    }

    // Force wheel drive joints unlimited (continuous)
    for(const char* name : {"left_wheel_drive_joint",
                             "right_wheel_drive_joint",
                             "rear_wheel_drive_joint"}) {
        auto* n = model->findJoint(name);
        if(n) n->joint.hasLimits = false;
    }

    // ── IK and hand nodes ─────────────────────────────────────────────────
    // Use hx5 hand palm as EE so IK positions the actual grasp point, not the wrist.
    // hx5_l_base is 78mm beyond arm_l_link7; targeting the palm makes grasping work.
    SceneNode* eeL   = ForwardKinematics::findByName(*model->root, "hx5_l_base");
    SceneNode* eeR   = ForwardKinematics::findByName(*model->root, "hx5_r_base");
    if(!eeL) eeL = ForwardKinematics::findByName(*model->root, "arm_l_link7");
    if(!eeR) eeR = ForwardKinematics::findByName(*model->root, "arm_r_link7");
    SceneNode* handL = eeL;
    SceneNode* handR = eeR;

    // Arm chain boundaries:
    //   shoulder = first arm joint (arm_l/r_link1) — workspace sphere center
    //   chainStop = parent of shoulder (arm_base_link) — IK traversal stops HERE (exclusive)
    SceneNode* shoulderL  = ForwardKinematics::findByName(*model->root, "arm_l_link1");
    SceneNode* shoulderR  = ForwardKinematics::findByName(*model->root, "arm_r_link1");
    SceneNode* chainStop  = ForwardKinematics::findByName(*model->root, "arm_base_link");

    // Theoretical max reach = sum of |localPos| from shoulder (arm_l/r_link1) to EE.
    // Using shoulderL/R as chainStop here so the shoulder's own parent-offset is excluded.
    float maxReachL = (eeL && shoulderL)
        ? InverseKinematics::computeChainLength(eeL, shoulderL) : 0.f;
    float maxReachR = (eeR && shoulderR)
        ? InverseKinematics::computeChainLength(eeR, shoulderR) : 0.f;
    {
        float measL = (eeL && chainStop)
            ? InverseKinematics::computeMaxReach(model->root.get(), eeL, chainStop) : 0.f;
        float measR = (eeR && chainStop)
            ? InverseKinematics::computeMaxReach(model->root.get(), eeR, chainStop) : 0.f;
        std::printf("[IK] reach  theory: L=%.3fm R=%.3fm  "
                    "joints-at-0: L=%.3fm R=%.3fm\n",
                    maxReachL, maxReachR, measL, measR);
    }

    // IK targets are in BASE-LOCAL space (follow the robot).
    // They will be properly initialized from FK when IK mode is first entered.
    // Set safe defaults now: arm roughly in front of base at shoulder height.
    jointPanel.ikTargetL = {0.2f,  0.8f, -0.2f};
    jointPanel.ikTargetR = {0.2f,  0.8f,  0.2f};

    // ── Wheel contact nodes (for ground-contact visualization) ───────────────
    SceneNode* wheelDriveL    = ForwardKinematics::findByName(*model->root, "left_wheel_drive");
    SceneNode* wheelDriveR    = ForwardKinematics::findByName(*model->root, "right_wheel_drive");
    SceneNode* wheelDriveRear = ForwardKinematics::findByName(*model->root, "rear_wheel_drive");

    // ── Wheel state ───────────────────────────────────────────────────────
    float driveAngleL = 0.f, driveAngleR = 0.f, driveAngleRear = 0.f;

    // ── Mouse drag state ──────────────────────────────────────────────────
    int  mouseDragObj  = -1;
    bool rightWasDown  = false;

    // Persistent base velocity/yaw for inertia/friction feel
    Vec3  baseVelCur   = {0.f, 0.f, 0.f};
    float baseYawRate  = 0.f;
    bool  lastIkMode   = false;   // detect FK→IK transition
    bool  cameraFollow = true;    // F key: camera tracks robot base

    using Clock = std::chrono::steady_clock;
    auto lastTime  = Clock::now();
    auto startTime = lastTime;   // wall-clock origin

    // Telemetry state
    float simTimeAcc   = 0.f;    // cumulative physics time (dt sum)
    float loopFreqEma  = 0.f;    // exponential moving average of loop frequency
    float ikErrL       = 0.f;    // IK residual error left arm
    float ikErrR       = 0.f;    // IK residual error right arm

    // ── Main loop ─────────────────────────────────────────────────────────
    while(!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto now = Clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        dt = std::min(dt, 0.05f);
        lastTime = now;

        // Telemetry update
        simTimeAcc += dt;
        float wallTimeSec = std::chrono::duration<float>(now - startTime).count();
        float instFreq    = (dt > 1e-6f) ? 1.0f / dt : 0.f;
        loopFreqEma = (loopFreqEma < 1.f)
                      ? instFreq   // cold start
                      : 0.97f * loopFreqEma + 0.03f * instFreq;

        bool wantCapture = io.WantCaptureKeyboard || io.WantCaptureMouse;
        input.update(dt, wantCapture);

        // ── Camera ────────────────────────────────────────────────────────
        if(input.keyPressed(GLFW_KEY_F)) cameraFollow = !cameraFollow;

        // When follow mode is on, camera target tracks robot visual base
        if(cameraFollow) {
            Vec3 wp = physics.basePosition();
            Vec3 visualBase = {wp.x, wp.y - kWheelGroundOfs + 0.8f, wp.z};
            renderer.camera.target = visualBase;  // aim at robot torso height
        }

        bool rightDown = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        if(!rightDown) {  // only orbit / pan when not dragging an object
            if(input.leftDragging())
                renderer.camera.onMouseDrag(input.mouseDX(), input.mouseDY());
            if(input.middleDragging()) {
                renderer.camera.onPan(input.mouseDX(), input.mouseDY());
                cameraFollow = false;  // manual pan disables follow
            }
        }
        renderer.camera.onScroll(input.scrollDY());

        // ── Mouse drag (right button) for cola can ────────────────────────
        {
            double mx, my;
            glfwGetCursorPos(window, &mx, &my);
            float ndcX = (float)mx / kWidth  * 2.f - 1.f;
            float ndcY = 1.f - (float)my / kHeight * 2.f;
            float aspect = (float)kWidth / kHeight;

            if(rightDown && !rightWasDown && !wantCapture) {
                // Rising edge: ray-pick
                Vec3 ro = renderer.camera.eye();
                Vec3 rd = renderer.camera.pickRay(ndcX, ndcY, aspect);
                mouseDragObj = physics.pickObject(ro, rd);
                if(mouseDragObj >= 0)
                    physics.beginMouseDrag(mouseDragObj);
            }

            if(rightDown && mouseDragObj >= 0) {
                // Project mouse ray onto y = tableTopY + canHalfH plane
                Vec3 ro = renderer.camera.eye();
                Vec3 rd = renderer.camera.pickRay(ndcX, ndcY, aspect);
                float targetY = kTableTopY + kCanHalfH;
                if(std::abs(rd.y) > 1e-4f) {
                    float t = (targetY - ro.y) / rd.y;
                    if(t > 0.f) {
                        Vec3 pos = {ro.x + rd.x*t, targetY, ro.z + rd.z*t};
                        // Clamp to table surface
                        float cx = kTablePos.x, cz = kTablePos.z;
                        float hx = kTableHalfExt.x - kCanRadius;
                        float hz = kTableHalfExt.z - kCanRadius;
                        pos.x = std::max(cx-hx, std::min(cx+hx, pos.x));
                        pos.z = std::max(cz-hz, std::min(cz+hz, pos.z));
                        physics.setMouseDragPosition(mouseDragObj, pos);
                    }
                }
            }

            if(!rightDown && rightWasDown && mouseDragObj >= 0) {
                physics.endMouseDrag(mouseDragObj);
                mouseDragObj = -1;
            }
            rightWasDown = rightDown;
        }

        // ── Sync robot base from previous physics step ────────────────────
        // We read last frame's physics result here (1-frame lag, imperceptible
        // at 60fps) so that hand sphere positions can be fed to Bullet BEFORE
        // physics.step() later in this frame → correct contact forces on can.
        {
            Vec3  wp  = physics.basePosition();
            float yaw = physics.baseYaw();
            if(model->root && !model->root->children.empty()) {
                auto* chassis = model->root->children[0].get();
                chassis->localPos = {wp.x, -wp.z, wp.y - kWheelGroundOfs};
                chassis->localRot = Quaternion::fromAxisAngle({0,0,1}, yaw);
            }
            model->update();
        }

        // ── Teleop ────────────────────────────────────────────────────────
        if(!wantCapture) {
            auto cmd = input.teleopCmd();

            // Wheel friction feel: accelerate on input, brake on release
            {
                constexpr float kAccel   = 3.0f;  // m/s² — how fast wheels spin up
                constexpr float kBrake   = 6.0f;  // m/s² — friction / rolling resistance
                constexpr float kMaxSpd  = 0.55f; // m/s cap
                Vec3 target = cmd.baseVelocity;    // 0 when no key held
                if(target.length() > 0.01f) {
                    // Drive: lerp current velocity toward target
                    float t = 1.f - std::exp(-kAccel * dt);
                    baseVelCur.x += (target.x - baseVelCur.x) * t;
                    baseVelCur.z += (target.z - baseVelCur.z) * t;
                    // Clamp to max speed
                    float spd = std::sqrt(baseVelCur.x*baseVelCur.x +
                                          baseVelCur.z*baseVelCur.z);
                    if(spd > kMaxSpd) {
                        baseVelCur.x = baseVelCur.x / spd * kMaxSpd;
                        baseVelCur.z = baseVelCur.z / spd * kMaxSpd;
                    }
                } else {
                    // In-place turn: instantly kill translation so rotation is clean.
                    if(std::abs(cmd.yawRate) > 0.01f) {
                        baseVelCur = {0.f, 0.f, 0.f};
                    } else {
                        // Coast-to-stop: exponential friction decay
                        float decay = std::exp(-kBrake * dt);
                        baseVelCur.x *= decay;
                        baseVelCur.z *= decay;
                        if(std::abs(baseVelCur.x) < 0.001f) baseVelCur.x = 0.f;
                        if(std::abs(baseVelCur.z) < 0.001f) baseVelCur.z = 0.f;
                    }
                }
            }
            // Yaw rate with fast-response inertia
            {
                float targetYaw = cmd.yawRate;
                if(std::abs(targetYaw) > 0.01f) {
                    baseYawRate += (targetYaw - baseYawRate) * (1.f - std::exp(-4.f*dt));
                } else {
                    baseYawRate *= std::exp(-10.f*dt);
                    if(std::abs(baseYawRate) < 0.01f) baseYawRate = 0.f;
                }
            }
            physics.setBaseVelocity(baseVelCur, baseYawRate, dt);

            // Wheel drive (continuous, no clamping)
            {
                float speed = baseVelCur.length();
                bool isPureTurn = (speed < 0.01f && std::abs(baseYawRate) > 0.01f);

                // Helper: clamp raw steer to ±90° range by flipping π and reversing drive.
                // Returns {clamped_steer, drive_sign}.
                auto clampSteer = [](float raw) -> std::pair<float,float> {
                    if(raw > 1.5707963f)  return {raw - 3.1415927f, -1.f};
                    if(raw < -1.5707963f) return {raw + 3.1415927f, -1.f};
                    return {raw, 1.f};
                };

                if(isPureTurn) {
                    // ── In-place turn: each wheel steers tangentially ──────────────────
                    // For a Y-up yaw at rate ω, the velocity at wheel at robot-local (px,pz) is:
                    //   v_x = ω * pz,  v_z = -ω * px
                    // Steer = atan2(-v_z, v_x).  Drive speed = |v|.
                    Vec3  physBP = physics.basePosition();
                    Vec3  vbase  = {physBP.x, physBP.y - kWheelGroundOfs, physBP.z};
                    Quaternion br = physics.baseOrientation();

                    // Compute robot-local position of a wheel drive node.
                    auto wLocal = [&](SceneNode* wd) -> Vec3 {
                        if(!wd) return {0.f, 0.f, 0.3f};
                        Vec3 ww = wd->worldTransform.transformPoint({0,0,0});
                        Vec3 d  = {ww.x - vbase.x, 0.f, ww.z - vbase.z};
                        return br.conjugate().rotate(d);
                    };

                    // Compute per-wheel steer+drive from its local position.
                    auto turnWheel = [&](SceneNode* driveNode,
                                         float& driveAngle,
                                         const char* jDrive,
                                         const char* jSteer) {
                        Vec3  p   = wLocal(driveNode);
                        float vx  = baseYawRate * p.z;
                        float vz  = -baseYawRate * p.x;
                        float spd = std::sqrt(vx*vx + vz*vz);

                        auto [st, ds] = clampSteer(std::atan2(-vz, vx));
                        driveAngle += ds * (spd / kWheelRadius) * dt;

                        if(auto* jd = model->findJoint(jDrive))  jd->joint.value = driveAngle;
                        if(auto* js = model->findJoint(jSteer))  js->joint.value = st;
                    };

                    turnWheel(wheelDriveL,    driveAngleL,    "left_wheel_drive_joint",  "left_wheel_steer_joint");
                    turnWheel(wheelDriveR,    driveAngleR,    "right_wheel_drive_joint", "right_wheel_steer_joint");
                    turnWheel(wheelDriveRear, driveAngleRear, "rear_wheel_drive_joint",  "rear_wheel_steer_joint");

                } else {
                    // ── Translation: all wheels share the same steer ───────────────────
                    // Steer joint range is ±1.58 rad (≈ ±90°).
                    // For directions beyond ±90° (e.g. pure-backward S key → rawSteer=π),
                    // flip steer by π and reverse drive direction.
                    float steer = 0.f;
                    float driveSign = 1.0f;
                    if(speed > 0.01f) {
                        auto [st, ds] = clampSteer(std::atan2(-baseVelCur.z, baseVelCur.x));
                        steer = st; driveSign = ds;
                    }

                    float dAngle = driveSign * (speed / kWheelRadius) * dt;
                    driveAngleL    += dAngle;
                    driveAngleR    += dAngle;
                    driveAngleRear += dAngle;

                    auto* jwL = model->findJoint("left_wheel_drive_joint");
                    auto* jwR = model->findJoint("right_wheel_drive_joint");
                    auto* jwB = model->findJoint("rear_wheel_drive_joint");
                    if(jwL) jwL->joint.value = driveAngleL;
                    if(jwR) jwR->joint.value = driveAngleR;
                    if(jwB) jwB->joint.value = driveAngleRear;

                    auto* jsL = model->findJoint("left_wheel_steer_joint");
                    auto* jsR = model->findJoint("right_wheel_steer_joint");
                    auto* jsB = model->findJoint("rear_wheel_steer_joint");
                    if(jsL) jsL->joint.value = steer;
                    if(jsR) jsR->joint.value = steer;
                    if(jsB) jsB->joint.value = steer;
                }
            }

            // Lift
            {
                auto* jl = model->findJoint("lift_joint");
                if(jl) { jl->joint.value += cmd.liftVelocity * dt; jl->joint.clamp(); }
            }

            // FK joint control
            if(!jointPanel.ikMode) {
                const auto& joints = jointPanel.joints();
                int n = (int)joints.size();
                int sel = n > 0 ? (input.selectedJointIndex() % n) : 0;
                jointPanel.selectedIndex = sel;
                if(n > 0) {
                    auto& j = joints[sel].node->joint;
                    j.value += cmd.selectedJointDelta;
                    j.clamp();
                }
            }

            // IK target keyboard control (base-local frame, 0.4 m/s)
            // I/K = +/-X (robot fwd/back), J/L = -/+Z (left/right strafe), U/O = +/-Y (up/down)
            // Hold 1 = left arm only, hold 2 = right arm only, neither = both arms
            if(jointPanel.ikMode) {
                const float kIkSpd = 0.4f;
                float step = kIkSpd * dt;
                Vec3 delta = {};
                if(input.keyDown(GLFW_KEY_I)) delta.x += step;
                if(input.keyDown(GLFW_KEY_K)) delta.x -= step;
                if(input.keyDown(GLFW_KEY_J)) delta.z -= step;
                if(input.keyDown(GLFW_KEY_L)) delta.z += step;
                if(input.keyDown(GLFW_KEY_U)) delta.y += step;
                if(input.keyDown(GLFW_KEY_O)) delta.y -= step;
                if(delta.x != 0.f || delta.y != 0.f || delta.z != 0.f) {
                    bool onlyR = input.keyDown(GLFW_KEY_2);
                    bool onlyL = input.keyDown(GLFW_KEY_1);
                    auto clampIK = [](Vec3& t) {
                        t.x = std::max(-1.5f, std::min(1.5f, t.x));
                        t.y = std::max(-0.5f, std::min(2.0f, t.y));
                        t.z = std::max(-1.5f, std::min(1.5f, t.z));
                    };
                    if(!onlyR) { jointPanel.ikTargetL += delta; clampIK(jointPanel.ikTargetL); }
                    if(!onlyL) { jointPanel.ikTargetR += delta; clampIK(jointPanel.ikTargetR); }
                }
            }

            if(input.keyPressed(GLFW_KEY_G))
                jointPanel.showGizmos = !jointPanel.showGizmos;

            // Z/X keys: toggle grip for L/R hand (press once = grip, press again = open)
            // Does NOT reset slider values when other keys are pressed.
            if(input.keyPressed(GLFW_KEY_Z)) {
                float next = (handPanel.fingerGrip[0] > 0.5f) ? 0.f : 1.f;
                handPanel.thumbGrip[0]  = next;
                handPanel.fingerGrip[0] = next;
            }
            if(input.keyPressed(GLFW_KEY_X)) {
                float next = (handPanel.fingerGrip[1] > 0.5f) ? 0.f : 1.f;
                handPanel.thumbGrip[1]  = next;
                handPanel.fingerGrip[1] = next;
            }

            if(input.keyDown(GLFW_KEY_ESCAPE))
                glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        // FK with base pose already set above; physics.step() moves to end of frame.

        // visual base position: physics box center adjusted by wheel-ground offset
        auto visualBasePos = [&]() -> Vec3 {
            Vec3 wp = physics.basePosition();
            return {wp.x, wp.y - kWheelGroundOfs, wp.z};
        };

        // ── IK mode transition: initialize targets from current FK EE pos ────
        if(jointPanel.ikMode && !lastIkMode) {
            Quaternion baseRot = physics.baseOrientation();
            Vec3       vbase   = visualBasePos();
            if(eeL) {
                Vec3 eeW = eeL->worldTransform.transformPoint({0,0,0});
                jointPanel.ikTargetL    = baseRot.conjugate().rotate(eeW - vbase);
                // Base-local orientation: q_base^{-1} * q_ee_world
                jointPanel.ikTargetRotL = (baseRot.conjugate()
                                           * eeL->worldTransform.extractRotation()).normalized();
            }
            if(eeR) {
                Vec3 eeW = eeR->worldTransform.transformPoint({0,0,0});
                jointPanel.ikTargetR    = baseRot.conjugate().rotate(eeW - vbase);
                jointPanel.ikTargetRotR = (baseRot.conjugate()
                                           * eeR->worldTransform.extractRotation()).normalized();
            }
        }
        // While in IK mode but orientation is not actively controlled, keep
        // the orientation target synced to current FK so enabling it later
        // never causes a sudden jump.
        if(jointPanel.ikMode && !jointPanel.ikUseOrientation) {
            Quaternion baseRot = physics.baseOrientation();
            if(eeL)
                jointPanel.ikTargetRotL = (baseRot.conjugate()
                    * eeL->worldTransform.extractRotation()).normalized();
            if(eeR)
                jointPanel.ikTargetRotR = (baseRot.conjugate()
                    * eeR->worldTransform.extractRotation()).normalized();
        }
        lastIkMode = jointPanel.ikMode;

        // ── IK ────────────────────────────────────────────────────────────
        if(jointPanel.ikMode) {
            // Convert base-local IK offsets → world targets (using visual base)
            Quaternion baseRot = physics.baseOrientation();
            Vec3       vbase   = visualBasePos();

            // When the user actively drags an RPY slider, snap the XYZ target
            // to the current EE position so the arm holds still while only
            // orientation is adjusted.
            if(jointPanel.ikRotChangedL && eeL) {
                Vec3 eeW = eeL->worldTransform.transformPoint({0,0,0});
                jointPanel.ikTargetL = baseRot.conjugate().rotate(eeW - vbase);
            }
            if(jointPanel.ikRotChangedR && eeR) {
                Vec3 eeW = eeR->worldTransform.transformPoint({0,0,0});
                jointPanel.ikTargetR = baseRot.conjugate().rotate(eeW - vbase);
            }

            Vec3 ikWorldL = baseRot.rotate(jointPanel.ikTargetL) + vbase;
            Vec3 ikWorldR = baseRot.rotate(jointPanel.ikTargetR) + vbase;

            // ── Workspace clamping: keep targets inside the reachable sphere ─
            // Shoulder world positions update each frame (robot can move/rotate).
            if(shoulderL && maxReachL > 0.01f) {
                Vec3 sW = shoulderL->worldTransform.transformPoint({0,0,0});
                ikWorldL = InverseKinematics::clampToWorkspace(ikWorldL, sW, maxReachL);
            }
            if(shoulderR && maxReachR > 0.01f) {
                Vec3 sW = shoulderR->worldTransform.transformPoint({0,0,0});
                ikWorldR = InverseKinematics::clampToWorkspace(ikWorldR, sW, maxReachR);
            }

            IKConfig cfg;
            cfg.useOrientation = jointPanel.ikUseOrientation;

            // Convert base-local orientation targets to world space
            Quaternion worldRotL = (baseRot * jointPanel.ikTargetRotL).normalized();
            Quaternion worldRotR = (baseRot * jointPanel.ikTargetRotR).normalized();

            // chainStop limits traversal to the 7 arm joints only
            // (prevents IK from accidentally moving lift, wheels, etc.)
            if(eeL) InverseKinematics::solve6(model->root.get(), eeL,
                        ikWorldL, worldRotL,
                        jointPanel.ikUseOrientation, cfg, chainStop);
            if(eeR) InverseKinematics::solve6(model->root.get(), eeR,
                        ikWorldR, worldRotR,
                        jointPanel.ikUseOrientation, cfg, chainStop);

            // Update world-space markers for renderer
            renderer.ikTargetL = ikWorldL;
            renderer.ikTargetR = ikWorldR;

            // IK residual error: distance from EE (after solve) to target
            model->update();  // compute updated world transforms
            auto errDist = [](SceneNode* ee, const Vec3& tgt) -> float {
                if(!ee) return 0.f;
                Vec3 p = ee->worldTransform.transformPoint({0,0,0});
                float dx=p.x-tgt.x, dy=p.y-tgt.y, dz=p.z-tgt.z;
                return std::sqrt(dx*dx + dy*dy + dz*dz);
            };
            ikErrL = errDist(eeL, ikWorldL);
            ikErrR = errDist(eeR, ikWorldR);
        } else {
            ikErrL = ikErrR = 0.f;
        }
        model->update();  // FK after IK

        // Apply finger/thumb angles from HandPanel, then re-run FK for hand
        HandPanel::applyToModel(*model,
            handPanel.thumbGrip[0],  handPanel.fingerGrip[0],
            handPanel.thumbGrip[1],  handPanel.fingerGrip[1]);
        model->update();  // FK after hand joints

        // ── Hand physics: update palm base ────────────────────────────────────
        // applyGripTorques() is driven by Bullet's internal pre-tick callback
        // (registered in HandPhysics::build) — fires once per physics substep
        // (500 Hz), not once per frame (60 Hz), so kp=20 stays stable.
        if(handPhysics.isBuilt()) {
            Mat4 palmLWorld = handL ? handL->worldTransform : Mat4::identity();
            Mat4 palmRWorld = handR ? handR->worldTransform : Mat4::identity();
            handPhysics.setPalmTransforms(palmLWorld, palmRWorld);
        }

        // ── Grasping (kinematic fallback when HandPhysics not built) ─────────
        {
            Vec3       palmPosL = handL ? handL->worldTransform.transformPoint({0,0,0}) : Vec3{};
            Vec3       palmPosR = handR ? handR->worldTransform.transformPoint({0,0,0}) : Vec3{};
            Quaternion palmRotL = handL ? handL->worldTransform.extractRotation()
                                        : Quaternion::identity();
            Quaternion palmRotR = handR ? handR->worldTransform.extractRotation()
                                        : Quaternion::identity();

            // When HandPhysics is active, Bullet friction handles grasping.
            // Kinematic attachment is only used as fallback.
            if(!handPhysics.isBuilt()) {
                physics.applyGripForce(0, handPanel.gripStrength(0), palmPosL, palmRotL, dt);
                physics.applyGripForce(1, handPanel.gripStrength(1), palmPosR, palmRotR, dt);
                physics.updateGraspedObjects(0, palmPosL, palmRotL);
                physics.updateGraspedObjects(1, palmPosR, palmRotR);
            }

            // Auto-drop: palm inside env box → release
            for(int side = 0; side < 2; ++side) {
                if(!physics.isGrasping(side)) continue;
                Vec3 palmW = (side == 0) ? palmPosL : palmPosR;
                for(const auto& sb : physics.staticBoxStates()) {
                    Vec3 lo = {sb.pos.x-sb.halfExtents.x, sb.pos.y-sb.halfExtents.y,
                               sb.pos.z-sb.halfExtents.z};
                    Vec3 hi = {sb.pos.x+sb.halfExtents.x, sb.pos.y+sb.halfExtents.y,
                               sb.pos.z+sb.halfExtents.z};
                    if(palmW.x>lo.x && palmW.x<hi.x &&
                       palmW.y>lo.y && palmW.y<hi.y &&
                       palmW.z>lo.z && palmW.z<hi.z) {
                        if(side==0) { handPanel.thumbGrip[0]=0.f; handPanel.fingerGrip[0]=0.f; }
                        else        { handPanel.thumbGrip[1]=0.f; handPanel.fingerGrip[1]=0.f; }
                        physics.applyGripForce(side, 0.f, palmW,
                            (side==0)?palmRotL:palmRotR, dt);
                        break;
                    }
                }
            }
        }

        // ── Sync arm collision bodies, step physics, read back fingers ───────
        robotCollider.update();
        physics.step(dt);

        // Copy Featherstone finger positions → SceneNode (renders fingers wrapping object)
        if(handPhysics.isBuilt()) {
            handPhysics.syncToFK();
            model->update();
        }

        // ── Sync renderer ─────────────────────────────────────────────────
        renderer.isGrounded    = physics.isGrounded();
        renderer.showGizmos    = jointPanel.showGizmos;
        renderer.showIkTargets = jointPanel.ikMode;
        if(!jointPanel.ikMode) {
            renderer.ikTargetL = {};
            renderer.ikTargetR = {};
        }
        renderer.eePosL = eeL ? eeL->worldTransform.transformPoint({0,0,0}) : Vec3{};
        renderer.eePosR = eeR ? eeR->worldTransform.transformPoint({0,0,0}) : Vec3{};

        // Hand proximity + grasp state → renderer
        renderer.handPosL  = handL ? handL->worldTransform.transformPoint({0,0,0}) : Vec3{};
        renderer.handPosR  = handR ? handR->worldTransform.transformPoint({0,0,0}) : Vec3{};
        renderer.graspDistL = physics.handNearestDist(0);
        renderer.graspDistR = physics.handNearestDist(1);
        renderer.graspingL  = physics.isGrasping(0);
        renderer.graspingR  = physics.isGrasping(1);
        renderer.graspRadius = PhysicsWorld::kGraspRadius;

        renderer.contactPts.clear(); // btMultiBody contact resolution is internal

        // Wheel ground-contact patches (project wheel drive node to Y=0 ground)
        {
            renderer.wheelContacts.clear();
            for(SceneNode* wd : {wheelDriveL, wheelDriveR, wheelDriveRear}) {
                if(!wd) continue;
                Vec3 ww = wd->worldTransform.transformPoint({0,0,0});
                renderer.wheelContacts.push_back({ww.x, 0.f, ww.z});
            }
        }

        // Update HandPanel per-frame status from physics
        for(int side = 0; side < 2; ++side) {
            handPanel.graspDist[side]  = physics.handNearestDist(side);
            handPanel.graspReady[side] = (handPanel.graspDist[side] <= PhysicsWorld::kGraspRadius);
            handPanel.isGrasping[side] = physics.isGrasping(side);
        }

        // Physics cylinder objects → renderer
        {
            auto states = physics.objectStates();
            renderer.objects.clear();
            for(const auto& s : states)
                renderer.objects.push_back({s.pos, s.rot, s.color,
                                             s.radius, s.halfHeight});
        }

        // ── ImGui ─────────────────────────────────────────────────────────
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if(jointPanel.draw(*model)) model->update();
        handPanel.draw();

        // ── Hand Physics tuning panel ─────────────────────────────────────────
        if(handPhysics.isBuilt()) {
            ImGui::SetNextWindowPos({10.f, (float)kHeight - 130.f}, ImGuiCond_Once);
            ImGui::SetNextWindowSize({260.f, 120.f}, ImGuiCond_Once);
            ImGui::Begin("Hand Physics");
            ImGui::SliderFloat("kp  (N·m/rad)",   &handPhysics.kp,       0.f,  30.f, "%.2f");
            ImGui::SliderFloat("kd  (N·m·s/rad)", &handPhysics.kd,       0.f,   1.0f, "%.3f");
            ImGui::SliderFloat("fMax (N·m)",       &handPhysics.forceMax, 0.f,   5.f, "%.2f");
            ImGui::TextDisabled("stable: kp<400, kd<0.9 (500 Hz)");
            ImGui::End();
        }

        // Key bindings / status overlay (top-right)
        {
            ImGui::SetNextWindowPos({(float)kWidth-210.f, 10.f}, ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.55f);
            ImGui::Begin("##hud", nullptr,
                         ImGuiWindowFlags_NoDecoration |
                         ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoNav);
            ImGui::Text("%.0f FPS", io.Framerate);
            ImGui::Separator();
            ImGui::Text("WASD     move (robot-local)");
            ImGui::Text("LR Arr   rotate base (in-place)");
            ImGui::Text("UD Arr   FK joint +/-");
            ImGui::Text("Q / E    lift up / down");
            ImGui::Text("Tab      next joint");
            ImGui::Text("Z / X    close L / R hand (hold)");
            ImGui::Text("G        gizmos  F  cam follow");
            ImGui::Text("RMB      drag can");
            if(jointPanel.ikMode) {
                ImGui::Separator();
                ImGui::TextColored({0.4f,0.9f,0.4f,1.f}, "IK keys (base-local):");
                ImGui::Text("I/K  EE fwd/back (+/-X)");
                ImGui::Text("J/L  EE left/right (-/+Z)");
                ImGui::Text("U/O  EE up/down (+/-Y)");
                ImGui::Text("Hold 1 = L only, 2 = R only");
            }
            ImGui::Separator();
            ImGui::Text("Ground:  %s", renderer.isGrounded ? "YES" : "no");
            ImGui::Text("Mode:    %s", jointPanel.ikMode ? "IK" : "FK");
            ImGui::Text("Yaw:     %.1f deg", physics.baseYaw() * 180.f / 3.14159f);
            ImGui::Separator();

            // Grasp proximity feedback (from HandPanel + physics)
            float dL = handPanel.graspDist[0], dR = handPanel.graspDist[1];
            float gr = PhysicsWorld::kGraspRadius;
            auto distColor = [gr](float d) -> ImVec4 {
                if(d <= gr)       return {0.2f,1.f,0.4f,1.f};  // green: in range
                if(d <= gr*2.f)   return {1.f,0.8f,0.1f,1.f};  // yellow: close
                return                   {1.f,0.4f,0.4f,1.f};  // red: far
            };
            auto graspLabel = [](float grip) { return grip > 0.01f ? "GRIPPING" : "open"; };
            if(handPanel.isGrasping[0])
                ImGui::TextColored({0.2f,1.f,0.4f,1.f}, "Hand L: HOLDING (grip=%.2f)",
                    handPanel.gripStrength(0));
            else if(dL > 9.f)  // no free object in range
                ImGui::TextColored({1.f,0.4f,0.4f,1.f}, "Hand L: ---  [%s]",
                    graspLabel(handPanel.fingerGrip[0]));
            else
                ImGui::TextColored(distColor(dL), "Hand L: %.3fm  %s  [%s]",
                    dL, dL <= gr ? "READY" : "far", graspLabel(handPanel.fingerGrip[0]));
            if(handPanel.isGrasping[1])
                ImGui::TextColored({0.2f,1.f,0.4f,1.f}, "Hand R: HOLDING (grip=%.2f)",
                    handPanel.gripStrength(1));
            else if(dR > 9.f)  // no free object in range
                ImGui::TextColored({1.f,0.4f,0.4f,1.f}, "Hand R: ---  [%s]",
                    graspLabel(handPanel.fingerGrip[1]));
            else
                ImGui::TextColored(distColor(dR), "Hand R: %.3fm  %s  [%s]",
                    dR, dR <= gr ? "READY" : "far", graspLabel(handPanel.fingerGrip[1]));

            if(mouseDragObj >= 0)
                ImGui::TextColored({1,0.7f,0,1}, "Dragging can");

            ImGui::Separator();
            ImGui::Text("Contact pts: %d", (int)renderer.contactPts.size());
            ImGui::End();
        }

        // ── Telemetry overlay (bottom-right) ──────────────────────────────
        {
            ImGuiIO& io2 = ImGui::GetIO();
            const float pad = 10.f;
            ImGui::SetNextWindowPos({io2.DisplaySize.x - pad, io2.DisplaySize.y - pad},
                                    ImGuiCond_Always, {1.f, 1.f});
            ImGui::SetNextWindowBgAlpha(0.60f);
            ImGui::Begin("##telem", nullptr,
                         ImGuiWindowFlags_NoDecoration |
                         ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoNav |
                         ImGuiWindowFlags_NoMove);

            // IK error (only meaningful in IK mode)
            if(jointPanel.ikMode) {
                float ikErr = std::max(ikErrL, ikErrR);
                ImVec4 errCol = ikErr < 0.005f ? ImVec4{0.2f,1.f,0.4f,1.f}
                              : ikErr < 0.02f  ? ImVec4{1.f,0.8f,0.1f,1.f}
                                               : ImVec4{1.f,0.3f,0.3f,1.f};
                ImGui::TextColored(errCol, "ik_err    [%.4f]", ikErr);
                ImGui::TextColored({0.7f,0.7f,0.7f,1.f},
                    "  L:%.4f  R:%.4f", ikErrL, ikErrR);
            } else {
                ImGui::TextColored({0.5f,0.5f,0.5f,1.f}, "ik_err    [--]");
            }
            ImGui::Separator();
            ImGui::Text("sim time    %.2fsec", simTimeAcc);
            ImGui::Text("wall time   %.2fsec", wallTimeSec);
            ImGui::Separator();
            ImGui::Text("Loop freq   [%.1f]Hz", instFreq);
            ImGui::Text("Loop freq (ema)  [%.1f]Hz", loopFreqEma);
            ImGui::End();
        }

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
