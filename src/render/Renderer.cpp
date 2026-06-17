#include "Renderer.hpp"
#include "io/STLLoader.hpp"
#include <epoxy/gl.h>
#include <stdexcept>
#include <cstdio>

// ── Grid shaders (embedded strings) ─────────────────────────────────────────
static const char* kGridVert = R"(
#version 430 core
layout(location=0) in vec3 aPos;
uniform mat4 uVP;
void main() { gl_Position = uVP * vec4(aPos,1.0); }
)";

static const char* kGridFrag = R"(
#version 430 core
out vec4 FragColor;
void main() { FragColor = vec4(0.35,0.35,0.35,1.0); }
)";

// ── Init ─────────────────────────────────────────────────────────────────────
bool Renderer::init(int width, int height) {
    m_width  = width;
    m_height = height;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.12f, 0.12f, 0.13f, 1.f);

    // Grid shader (embedded)
    m_grid.loadSources(kGridVert, kGridFrag);

    buildGrid(5.f, 20);
    return true;
}

void Renderer::resize(int w, int h) {
    m_width = w; m_height = h;
    glViewport(0, 0, w, h);
}

void Renderer::shutdown() {
    if(m_gridVAO) glDeleteVertexArrays(1, &m_gridVAO);
    if(m_gridVBO) glDeleteBuffers(1, &m_gridVBO);
}

// ── Grid ─────────────────────────────────────────────────────────────────────
void Renderer::buildGrid(float size, int div) {
    std::vector<float> pts;
    float step = 2.f * size / div;
    for(int i = 0; i <= div; ++i) {
        float v = -size + i * step;
        // X-aligned line
        pts.insert(pts.end(), {-size, 0.f, v,  size, 0.f, v});
        // Z-aligned line
        pts.insert(pts.end(), {v, 0.f, -size,  v, 0.f, size});
    }
    m_gridLines = static_cast<int>(pts.size() / 3);

    glGenVertexArrays(1, &m_gridVAO);
    glGenBuffers(1, &m_gridVBO);
    glBindVertexArray(m_gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(pts.size() * sizeof(float)),
                 pts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), nullptr);
    glBindVertexArray(0);
}

void Renderer::drawGrid() {
    if(!m_gridVAO || !m_grid.valid()) return;
    float aspect = (float)m_width / (float)m_height;
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
            STLMesh stl = STLLoader::load(model.meshPaths[i]);
            m_meshes[i].upload(stl);
        } catch(const std::exception& e) {
            std::fprintf(stderr, "[Renderer] Mesh load failed: %s\n", e.what());
        }
    }
}

// ── Per-node draw ─────────────────────────────────────────────────────────────
void Renderer::drawNode(const SceneNode& node,
                        const Mat4& view, const Mat4& proj)
{
    if(node.meshIndex >= 0 &&
       node.meshIndex < (int)m_meshes.size() &&
       m_meshes[node.meshIndex].valid())
    {
        m_phong.use();
        Mat4 model = node.worldTransform * Mat4::scaling(node.meshScale);
        m_phong.setMat4("uModel", model);
        m_phong.setMat4("uView",  view);
        m_phong.setMat4("uProj",  proj);
        m_phong.setMat4("uNormal", node.worldTransform.normalMatrix());
        m_phong.setVec3("uObjectColor", node.color);
        m_phong.setVec3("uLightPos", {5.f, 8.f, 5.f});
        m_phong.setVec3("uViewPos",  camera.eye());

        m_meshes[node.meshIndex].draw();
        m_phong.unuse();
    }

    for(const auto& child : node.children)
        drawNode(*child, view, proj);
}

// ── Main render ───────────────────────────────────────────────────────────────
void Renderer::render(RobotModel& model) {
    // Lazy mesh upload
    if(m_meshes.size() != model.meshPaths.size())
        loadMeshes(model);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float aspect = (float)m_width / (float)m_height;
    Mat4 view = camera.view();
    Mat4 proj = camera.projection(aspect);

    drawGrid();

    if(model.root) {
        if(!m_phong.valid()) {
            m_phong.loadFiles("assets/shaders/phong.vert",
                              "assets/shaders/phong.frag");
        }
        drawNode(*model.root, view, proj);
    }
}
