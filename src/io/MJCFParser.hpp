#pragma once
#include "robot/RobotModel.hpp"
#include <string>
#include <memory>

// Parses a MuJoCo MJCF XML file into a RobotModel.
// Supports: body hierarchy, hinge/slide/free joints, mesh geoms.
// Does NOT parse actuators (joint limits come from the joint element itself).
class MJCFParser {
public:
    // baseDir: directory containing the XML (for resolving relative mesh paths)
    static std::unique_ptr<RobotModel> parse(const std::string& xmlPath,
                                              const std::string& baseDir = "");
};
