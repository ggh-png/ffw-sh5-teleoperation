#pragma once
#include "robot/RobotModel.hpp"
#include <string>
#include <memory>

// Global physics options extracted from MJCF <option> element.
// All values are in Y-up world space (MJCF Z-up is converted on import).
struct PhysicsOptions {
    float gravity  = -9.81f;   // m/s² — world Y axis (MJCF gz converted)
    float timestep = 0.002f;   // s    — integrator timestep (MuJoCo default)
    float impratio = 1.f;      // impedance ratio (affects constraint softness)
};

// Combined result of parsing a single MJCF file.
struct ParseResult {
    std::unique_ptr<RobotModel> model;
    PhysicsOptions              physics;
};

// Parses a MuJoCo MJCF XML file into a RobotModel + physics options.
// Supports: body hierarchy, hinge/slide/free joints, mesh geoms,
//           <inertial>, <default> joint/actuator dynamics, geom friction,
//           and <option> global settings.
class MJCFParser {
public:
    // baseDir: directory containing the XML (for resolving relative mesh paths).
    // Returns both the scene graph and the physics configuration.
    static ParseResult parse(const std::string& xmlPath,
                             const std::string& baseDir = "");
};
