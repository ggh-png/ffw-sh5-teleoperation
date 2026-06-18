#include "Renderer.hpp"
#include "io/STLLoader.hpp"
#include <epoxy/gl.h>
#include <cstdio>
#include <cmath>
#include <stdexcept>

// ── Embedded shaders ─────────────────────────────────────────────────────────

static const char* kGridVert = R"(
#version 430 core
layout(location=0) in vec3 aPos;
uniform mat4 uVP;
void main() { gl_Position = uVP * vec4(aPos,1.0); }
)";
static const char* kGridFrag = R"(
#version 430 core
out vec4 FragColor;
void main() { FragColor = vec4(0.28,0.28,0.28,1.0); }
)";

static const char* kLineVert = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
out vec3 vColor;
uniform mat4 uVP;
void main() { vColor = aColor; gl_Position = uVP * vec4(aPos,1.0); }
)";
static const char* kLineFrag = R"(
#version 430 core
in vec3 vColor;
out vec4 FragColor;
void main() { FragColor = vec4(vColor, 1.0); }
)";

// ── Init ─────────────────────────────────────────────────────────────────────
bool Renderer::init(int width, int height) {
    m_width  = width;
    m_height = height;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.12f, 0.12f, 0.13f, 1.f);

    m_grid.loadSources(kGridVert, kGridFrag);
    m_line.loadSources(kLineVert, kLineFrag);

    buildGrid(5.f, 20);
    initGizmoBuffers();
    initShadowMap();
    buildCylinderMesh();
    buildBoxMesh();
    return true;
}

void Renderer::resize(int w, int h) {
    m_width = w; m_height = h;
    glViewport(0, 0, w, h);
}

void Renderer::shutdown() {
    if(m_gridVAO)  glDeleteVertexArrays(1, &m_gridVAO);
    if(m_gridVBO)  glDeleteBuffers(1, &m_gridVBO);
    if(m_gizVAO)   glDeleteVertexArrays(1, &m_gizVAO);
    if(m_gizVBO)   glDeleteBuffers(1, &m_gizVBO);
    if(m_shadowFBO) glDeleteFramebuffers(1, &m_shadowFBO);
    if(m_shadowTex) glDeleteTextures(1, &m_shadowTex);
}

// ── Shadow map ────────────────────────────────────────────────────────────────
void Renderer::initShadowMap() {
    glGenTextures(1, &m_shadowTex);
    glBindTexture(GL_TEXTURE_2D, m_shadowTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F,
                 kShadowW, kShadowH, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float border[] = {1.f,1.f,1.f,1.f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenFramebuffers(1, &m_shadowFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_shadowFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, m_shadowTex, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::fprintf(stderr, "[Renderer] Shadow FBO incomplete!\n");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ── Cylinder mesh (unit: r=1, halfH=1, axis=Y) ───────────────────────────────
void Renderer::buildCylinderMesh() {
    STLMesh m;
    const int   N   = 20;
    const float pi2 = 2.f * 3.14159265f;

    for(int i = 0; i < N; ++i) {
        float a0=pi2*i/N, a1=pi2*(i+1)/N;
        float c0=std::cos(a0), s0=std::sin(a0);
        float c1=std::cos(a1), s1=std::sin(a1);

        Vec3 bl={c0,-1,s0}, br={c1,-1,s1}, tl={c0,1,s0}, tr={c1,1,s1};
        Vec3 nl={c0,0,s0}, nr={c1,0,s1};

        // Side (CCW from outside): br,bl,tl  and  br,tl,tr
        m.vertices.insert(m.vertices.end(),{br,bl,tl,br,tl,tr});
        m.normals.insert(m.normals.end(),  {nr,nl,nl,nr,nl,nr});

        // Top cap (+Y normal): center, tr, tl
        m.vertices.insert(m.vertices.end(),{{0,1,0},tr,tl});
        m.normals.insert(m.normals.end(),  {{0,1,0},{0,1,0},{0,1,0}});

        // Bottom cap (-Y normal): center, bl, br
        m.vertices.insert(m.vertices.end(),{{0,-1,0},bl,br});
        m.normals.insert(m.normals.end(),  {{0,-1,0},{0,-1,0},{0,-1,0}});
    }
    for(uint32_t k=0;k<(uint32_t)m.vertices.size();++k) m.indices.push_back(k);
    m_cylinder.upload(m);
}

// ── Box mesh (unit cube: -1,-1,-1 → 1,1,1) ───────────────────────────────────
void Renderer::buildBoxMesh() {
    STLMesh m;
    auto addFace = [&](const Vec3& n,
                       const Vec3& v0, const Vec3& v1,
                       const Vec3& v2, const Vec3& v3) {
        // Two CCW triangles forming the face
        for(const Vec3& v : {v0,v1,v2, v0,v2,v3}) {
            m.vertices.push_back(v);
            m.normals.push_back(n);
        }
    };
    // Each face: vertices ordered CCW when viewed from the normal direction
    addFace({0,1,0},  {-1,1,-1},{-1,1,1},{1,1,1},{1,1,-1});   // +Y
    addFace({0,-1,0}, {1,-1,-1},{1,-1,1},{-1,-1,1},{-1,-1,-1}); // -Y
    addFace({1,0,0},  {1,-1,-1},{1,1,-1},{1,1,1},{1,-1,1});     // +X
    addFace({-1,0,0}, {-1,-1,1},{-1,1,1},{-1,1,-1},{-1,-1,-1}); // -X
    addFace({0,0,1},  {1,-1,1},{1,1,1},{-1,1,1},{-1,-1,1});     // +Z
    addFace({0,0,-1}, {-1,-1,-1},{-1,1,-1},{1,1,-1},{1,-1,-1}); // -Z
    for(uint32_t k=0;k<(uint32_t)m.vertices.size();++k) m.indices.push_back(k);
    m_box.upload(m);
}

// ── Grid ─────────────────────────────────────────────────────────────────────
void Renderer::buildGrid(float size, int div) {
    std::vector<float> pts;
    float step = 2.f * size / div;
    for(int i = 0; i <= div; ++i) {
        float v = -size + i * step;
        pts.insert(pts.end(), {-size,0,v,  size,0,v});
        pts.insert(pts.end(), {v,0,-size,  v,0,size});
    }
    m_gridLines = (int)(pts.size() / 3);
    glGenVertexArrays(1, &m_gridVAO);
    glGenBuffers(1, &m_gridVBO);
    glBindVertexArray(m_gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(pts.size()*sizeof(float)),
                 pts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),nullptr);
    glBindVertexArray(0);
}

void Renderer::drawGrid() {
    if(!m_gridVAO || !m_grid.valid()) return;
    float aspect = (float)m_width / m_height;
    Mat4 vp = camera.projection(aspect) * camera.view();
    m_grid.use();
    m_grid.setMat4("uVP", vp);
    glBindVertexArray(m_gridVAO);
    glDrawArrays(GL_LINES, 0, m_gridLines);
    glBindVertexArray(0);
    m_grid.unuse();
}

// ── Mesh loading ─────────────────────────────────────────────────────────────
void Renderer::loadMeshes(RobotModel& model) {
    m_meshes.clear();
    m_meshes.resize(model.meshPaths.size());
    for(size_t i = 0; i < model.meshPaths.size(); ++i) {
        try {
            m_meshes[i].upload(STLLoader::load(model.meshPaths[i]));
        } catch(const std::exception& e) {
            std::fprintf(stderr, "[Renderer] Mesh %zu: %s\n", i, e.what());
        }
    }
}

// ── Robot node drawing ────────────────────────────────────────────────────────
void Renderer::drawNode(const SceneNode& node, const Mat4& view, const Mat4& proj) {
    if(node.meshIndex >= 0 && node.meshIndex < (int)m_meshes.size() &&
       m_meshes[node.meshIndex].valid()) {
        Mat4 mdl = node.worldTransform * Mat4::scaling(node.meshScale);
        m_phong.setMat4("uModel", mdl);
        m_phong.setMat4("uNormal", mdl.normalMatrix());
        m_phong.setVec3("uObjectColor", node.color);
        m_meshes[node.meshIndex].draw();
    }
    for(const auto& ch : node.children) drawNode(*ch, view, proj);
}

void Renderer::drawNodeShadow(const SceneNode& node) {
    if(node.meshIndex >= 0 && node.meshIndex < (int)m_meshes.size() &&
       m_meshes[node.meshIndex].valid()) {
        Mat4 mdl = node.worldTransform * Mat4::scaling(node.meshScale);
        m_shadow.setMat4("uModel", mdl);
        m_meshes[node.meshIndex].draw();
    }
    for(const auto& ch : node.children) drawNodeShadow(*ch);
}

// ── Dynamic cylinders ─────────────────────────────────────────────────────────
void Renderer::drawObjects(const Mat4& view, const Mat4& proj) {
    if(!m_cylinder.valid()) return;
    for(const auto& o : objects) {
        Mat4 mdl = Mat4::translation(o.pos)
                 * Mat4::fromQuaternion(o.rot)
                 * Mat4::scaling({o.radius, o.halfHeight, o.radius});
        m_phong.setMat4("uModel", mdl);
        m_phong.setMat4("uNormal", mdl.normalMatrix());
        m_phong.setVec3("uObjectColor", o.color);
        m_cylinder.draw();
    }
}
void Renderer::drawObjectsShadow() {
    if(!m_cylinder.valid()) return;
    for(const auto& o : objects) {
        Mat4 mdl = Mat4::translation(o.pos)
                 * Mat4::fromQuaternion(o.rot)
                 * Mat4::scaling({o.radius, o.halfHeight, o.radius});
        m_shadow.setMat4("uModel", mdl);
        m_cylinder.draw();
    }
}

// ── Static boxes ──────────────────────────────────────────────────────────────
void Renderer::drawBoxes(const Mat4& view, const Mat4& proj) {
    if(!m_box.valid()) return;
    for(const auto& b : boxes) {
        Mat4 mdl = Mat4::translation(b.pos)
                 * Mat4::fromQuaternion(b.rot)
                 * Mat4::scaling(b.halfExtents);
        m_phong.setMat4("uModel", mdl);
        m_phong.setMat4("uNormal", mdl.normalMatrix());
        m_phong.setVec3("uObjectColor", b.color);
        m_box.draw();
    }
}
void Renderer::drawBoxesShadow() {
    if(!m_box.valid()) return;
    for(const auto& b : boxes) {
        Mat4 mdl = Mat4::translation(b.pos)
                 * Mat4::fromQuaternion(b.rot)
                 * Mat4::scaling(b.halfExtents);
        m_shadow.setMat4("uModel", mdl);
        m_box.draw();
    }
}

// ── Gizmo buffers ─────────────────────────────────────────────────────────────
void Renderer::initGizmoBuffers() {
    glGenVertexArrays(1, &m_gizVAO);
    glGenBuffers(1, &m_gizVBO);
    glBindVertexArray(m_gizVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_gizVBO);
    glBufferData(GL_ARRAY_BUFFER, 1024*6*sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

void Renderer::drawGizmos(RobotModel& model, const Mat4& vp) {
    if(!m_line.valid() || !m_gizVAO) return;
    auto nodes = model.allNodes();
    std::vector<float> data;
    data.reserve(nodes.size() * 18);
    for(const auto* n : nodes) {
        Vec3 o  = n->worldTransform.transformPoint({0,0,0});
        Vec3 ax = n->worldTransform.transformDir({1,0,0}).normalized() * gizmoScale;
        Vec3 ay = n->worldTransform.transformDir({0,1,0}).normalized() * gizmoScale;
        Vec3 az = n->worldTransform.transformDir({0,0,1}).normalized() * gizmoScale;
        data.insert(data.end(),{o.x,o.y,o.z,1,0,0});
        data.insert(data.end(),{o.x+ax.x,o.y+ax.y,o.z+ax.z,1,0,0});
        data.insert(data.end(),{o.x,o.y,o.z,0,1,0});
        data.insert(data.end(),{o.x+ay.x,o.y+ay.y,o.z+ay.z,0,1,0});
        data.insert(data.end(),{o.x,o.y,o.z,0,0,1});
        data.insert(data.end(),{o.x+az.x,o.y+az.y,o.z+az.z,0,0,1});
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_gizVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(data.size()*sizeof(float)), data.data());
    m_line.use();
    m_line.setMat4("uVP", vp);
    glDisable(GL_DEPTH_TEST); glLineWidth(2.f);
    glBindVertexArray(m_gizVAO);
    glDrawArrays(GL_LINES, 0, (GLsizei)(data.size()/6));
    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST); glLineWidth(1.f);
    m_line.unuse();
}

// ── Overlay lines ─────────────────────────────────────────────────────────────
static void pushLine(std::vector<float>& d, const Vec3& a, const Vec3& b, const Vec3& c) {
    d.insert(d.end(),{a.x,a.y,a.z,c.x,c.y,c.z});
    d.insert(d.end(),{b.x,b.y,b.z,c.x,c.y,c.z});
}
static void pushCross(std::vector<float>& d, const Vec3& p, float s, const Vec3& c) {
    pushLine(d,{p.x-s,p.y,p.z},{p.x+s,p.y,p.z},c);
    pushLine(d,{p.x,p.y-s,p.z},{p.x,p.y+s,p.z},c);
    pushLine(d,{p.x,p.y,p.z-s},{p.x,p.y,p.z+s},c);
}
static void pushDiamond(std::vector<float>& d, const Vec3& p, const Vec3& col) {
    float r = 0.04f;
    Vec3 pts[4]={{p.x+r,0,p.z},{p.x,0,p.z+r},{p.x-r,0,p.z},{p.x,0,p.z-r}};
    for(int i=0;i<4;++i) pushLine(d,pts[i],pts[(i+1)%4],col);
    pushLine(d, p, {p.x,0,p.z}, col);
}

// Circle in XZ plane at given center, radius, N segments
static void pushCircleXZ(std::vector<float>& d, const Vec3& c, float r,
                          int N, const Vec3& col) {
    const float pi2 = 6.28318530f;
    for(int i = 0; i < N; ++i) {
        float a0 = pi2 * i / N, a1 = pi2 * (i+1) / N;
        Vec3 p0 = {c.x + r*std::cos(a0), c.y, c.z + r*std::sin(a0)};
        Vec3 p1 = {c.x + r*std::cos(a1), c.y, c.z + r*std::sin(a1)};
        pushLine(d, p0, p1, col);
    }
}



void Renderer::drawOverlays(const Mat4& vp) {
    if(!m_line.valid() || !m_gizVAO) return;
    std::vector<float> data;

    if(showIkTargets) {
        // IK target markers — green (L) / cyan (R)
        pushCross(data, ikTargetL, 0.07f, {0.1f,0.9f,0.1f});
        pushCross(data, ikTargetR, 0.07f, {0.1f,0.9f,0.9f});
        pushDiamond(data, ikTargetL, {0.1f,0.7f,0.1f});
        pushDiamond(data, ikTargetR, {0.1f,0.7f,0.7f});
        // Current actual EE — orange (L) / yellow (R)
        pushCross(data, eePosL, 0.05f, {1.0f,0.5f,0.0f});
        pushCross(data, eePosR, 0.05f, {1.0f,0.9f,0.0f});
    }

    // Grasp proximity rings removed — grasping is now purely contact-based.

    // ── Wheel contact patches ─────────────────────────────────────────────
    // Each wheel: contact ellipse on the ground + vertical line from wheel center.
    // Color: bright cyan when grounded, grey when not.
    {
        Vec3 wcol = isGrounded ? Vec3{0.2f, 0.9f, 1.0f} : Vec3{0.5f, 0.5f, 0.5f};
        for(const auto& wc : wheelContacts) {
            // Contact patch ellipse (tire deforms slightly, represent as oval)
            pushCircleXZ(data, wc, 0.04f, 12, wcol);         // contact patch radius
            pushCircleXZ(data, wc, wheelRadius, 16, {wcol.x*0.5f, wcol.y*0.5f, wcol.z*0.5f});
            // Cross at contact center
            pushLine(data, {wc.x-0.03f,0,wc.z}, {wc.x+0.03f,0,wc.z}, wcol);
            pushLine(data, {wc.x,0,wc.z-0.03f}, {wc.x,0,wc.z+0.03f}, wcol);
            // Vertical line from contact to wheel center
            Vec3 wTop = {wc.x, wheelRadius, wc.z};
            pushLine(data, wc, wTop, {wcol.x*0.6f, wcol.y*0.6f, wcol.z*0.6f});
        }
    }

    // ── Contact / friction point visualization ────────────────────────────
    // Each contact manifold point: dot + contact normal arrow.
    // color by type:  0=ground (blue), 1=obj-obj (white), 2=hand-obj (magenta)
    {
        static const Vec3 kTypeColor[3] = {
            {0.3f, 0.5f, 1.0f},   // 0: ground contact — blue
            {0.9f, 0.9f, 0.9f},   // 1: obj-obj        — white
            {1.0f, 0.3f, 1.0f},   // 2: hand-obj       — magenta
        };
        for(const auto& cp : contactPts) {
            int t = (cp.type >= 0 && cp.type < 3) ? cp.type : 1;
            Vec3 col = kTypeColor[t];
            // Small diamond cross at contact point
            pushCross(data, cp.pos, 0.015f, col);
            // Normal arrow (scaled by penetration depth, min 2cm for visibility)
            float nLen = std::max(cp.depth * 5.f, 0.025f);
            Vec3 tip = {cp.pos.x + cp.normal.x*nLen,
                        cp.pos.y + cp.normal.y*nLen,
                        cp.pos.z + cp.normal.z*nLen};
            pushLine(data, cp.pos, tip, col);
        }
    }

    if(data.empty()) return;

    // Resize buffer if needed (initial allocation was 1024 lines)
    std::size_t needed = data.size() * sizeof(float);
    static std::size_t s_bufSz = 1024*6*sizeof(float);
    if(needed > s_bufSz) {
        s_bufSz = needed * 2;
        glBindBuffer(GL_ARRAY_BUFFER, m_gizVBO);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)s_bufSz, nullptr, GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_gizVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(data.size()*sizeof(float)), data.data());
    m_line.use();
    m_line.setMat4("uVP", vp);
    glDisable(GL_DEPTH_TEST); glLineWidth(2.f);
    glBindVertexArray(m_gizVAO);
    glDrawArrays(GL_LINES, 0, (GLsizei)(data.size()/6));
    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST); glLineWidth(1.f);
    m_line.unuse();
}

// ── Main render ───────────────────────────────────────────────────────────────
void Renderer::render(RobotModel& model) {
    if(m_meshes.size() != model.meshPaths.size()) loadMeshes(model);

    if(!m_phong.valid())
        m_phong.loadFiles("assets/shaders/phong.vert","assets/shaders/phong.frag");
    if(!m_shadow.valid())
        m_shadow.loadFiles("assets/shaders/shadow.vert","assets/shaders/shadow.frag");

    const Vec3 kLightPos = {5.f, 8.f, 5.f};

    // ── Light-space matrix ────────────────────────────────────────────────
    m_lightSpaceMat = Mat4::ortho(-8.f,8.f,-8.f,8.f, 0.5f,35.f)
                    * Mat4::lookAt(kLightPos,{0,0,0},{0,1,0});

    // ── Shadow pass ───────────────────────────────────────────────────────
    if(m_shadowFBO && m_shadow.valid()) {
        glViewport(0,0,kShadowW,kShadowH);
        glBindFramebuffer(GL_FRAMEBUFFER, m_shadowFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        glCullFace(GL_FRONT);
        m_shadow.use();
        m_shadow.setMat4("uLightSpace", m_lightSpaceMat);
        if(model.root) drawNodeShadow(*model.root);
        drawObjectsShadow();
        drawBoxesShadow();
        m_shadow.unuse();
        glCullFace(GL_BACK);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0,0,m_width,m_height);
    }

    // ── Main colour pass ──────────────────────────────────────────────────
    glClearColor(isGrounded ? 0.10f : 0.12f,
                 isGrounded ? 0.13f : 0.12f,
                 isGrounded ? 0.10f : 0.13f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float aspect = (float)m_width / m_height;
    Mat4 view = camera.view();
    Mat4 proj = camera.projection(aspect);
    Mat4 vp   = proj * view;

    // Draw grid with polygon offset so floor doesn't z-fight
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.f, 1.f);
    drawGrid();
    glDisable(GL_POLYGON_OFFSET_FILL);

    if(m_phong.valid()) {
        m_phong.use();
        m_phong.setMat4("uView",       view);
        m_phong.setMat4("uProj",       proj);
        m_phong.setMat4("uLightSpace", m_lightSpaceMat);
        m_phong.setVec3("uLightPos",   kLightPos);
        m_phong.setVec3("uViewPos",    camera.eye());
        m_phong.setInt ("uShadowMap",  0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_shadowTex);

        drawBoxes(view, proj);           // floor + table (rendered first for depth)
        if(model.root) drawNode(*model.root, view, proj);
        drawObjects(view, proj);         // cola cans

        glBindTexture(GL_TEXTURE_2D, 0);
        m_phong.unuse();
    }

    if(showGizmos) drawGizmos(model, vp);
    drawOverlays(vp);
}
