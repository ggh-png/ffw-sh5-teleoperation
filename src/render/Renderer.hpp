#pragma once
#include "ShaderProgram.hpp"
#include "Mesh.hpp"
#include "Camera.hpp"
#include "robot/RobotModel.hpp"
#include <vector>
#include <memory>

class Renderer {
public:
    Camera camera;

    bool init(int width, int height);
    void resize(int width, int height);
    void render(RobotModel& model);
    void shutdown();

private:
    int m_width  = 1280;
    int m_height = 720;

    ShaderProgram           m_phong;
    ShaderProgram           m_grid;
    std::vector<Mesh>       m_meshes;    // indexed by RobotModel::meshPaths

    GLuint m_gridVAO = 0;
    GLuint m_gridVBO = 0;
    int    m_gridLines = 0;

    void loadMeshes(RobotModel& model);
    void drawGrid();
    void drawNode(const SceneNode& node, const Mat4& view, const Mat4& proj);
    void buildGrid(float size, int divisions);
};
