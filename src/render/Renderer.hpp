#pragma once
#include "ShaderProgram.hpp"
#include "Mesh.hpp"
#include "Camera.hpp"
#include "robot/RobotModel.hpp"
#include <vector>
#include <memory>

// Dynamic cylindrical object (cola can)
struct RenderObj {
    Vec3       pos;
    Quaternion rot;
    Vec3       color      = {0.8f, 0.15f, 0.15f};
    float      radius     = 0.033f;
    float      halfHeight = 0.06f;
};

// Static box (table, floor, shelf, etc.)
struct RenderBox {
    Vec3       pos;
    Quaternion rot  = Quaternion::identity();
    Vec3       color;
    Vec3       halfExtents;
};

class Renderer {
public:
    Camera camera;

    bool  showGizmos  = false;
    float gizmoScale  = 0.05f;
    bool  isGrounded  = false;

    Vec3  ikTargetL   = {0,0,0};   // IK target  → green  cross
    Vec3  ikTargetR   = {0,0,0};
    Vec3  eePosL      = {0,0,0};   // current EE → orange cross
    Vec3  eePosR      = {0,0,0};
    bool  showIkTargets = false;

    // Grasp proximity feedback (set every frame from physics)
    Vec3  handPosL     = {0,0,0};
    Vec3  handPosR     = {0,0,0};
    float graspDistL   = 1e9f;  // hand-L to nearest object distance
    float graspDistR   = 1e9f;
    bool  graspingL    = false;
    bool  graspingR    = false;
    float graspRadius  = 0.18f; // must match PhysicsWorld::kGraspRadius

    // Contact / friction visualization (from Bullet manifolds)
    struct ContactPt { Vec3 pos; Vec3 normal; float depth; int type; };
    std::vector<ContactPt> contactPts;

    // Wheel ground-contact patches (one per wheel, Y=0 ground level)
    std::vector<Vec3> wheelContacts;   // center of wheel on ground
    float             wheelRadius = 0.09f;

    std::vector<RenderObj> objects;  // dynamic cylinders
    std::vector<RenderBox> boxes;    // static/floor boxes

    bool init(int width, int height);
    void resize(int width, int height);
    void render(RobotModel& model);
    void shutdown();

private:
    static constexpr int kShadowW = 2048;
    static constexpr int kShadowH = 2048;

    int m_width  = 1280;
    int m_height = 720;

    ShaderProgram m_phong;
    ShaderProgram m_shadow;
    ShaderProgram m_grid;
    ShaderProgram m_line;

    std::vector<Mesh> m_meshes;
    Mesh              m_cylinder;   // unit cylinder (r=1, halfH=1)
    Mesh              m_box;        // unit box (-1,-1,-1)→(1,1,1)

    GLuint m_shadowFBO = 0;
    GLuint m_shadowTex = 0;
    Mat4   m_lightSpaceMat;

    GLuint m_gridVAO = 0;
    GLuint m_gridVBO = 0;
    int    m_gridLines = 0;

    GLuint m_gizVAO = 0;
    GLuint m_gizVBO = 0;

    void loadMeshes(RobotModel& model);
    void buildGrid(float size, int divisions);
    void initGizmoBuffers();
    void initShadowMap();
    void buildCylinderMesh();
    void buildBoxMesh();

    void drawGrid();
    void drawNode(const SceneNode& node, const Mat4& view, const Mat4& proj);
    void drawNodeShadow(const SceneNode& node);
    void drawObjects(const Mat4& view, const Mat4& proj);
    void drawObjectsShadow();
    void drawBoxes(const Mat4& view, const Mat4& proj);
    void drawBoxesShadow();
    void drawGizmos(RobotModel& model, const Mat4& vp);
    void drawOverlays(const Mat4& vp);
};
