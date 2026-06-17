#include "MJCFParser.hpp"
#include "math/Math.hpp"
#include <tinyxml2.h>
#include <sstream>
#include <unordered_map>
#include <stdexcept>
#include <cstring>
#include <filesystem>

namespace fs = std::filesystem;
using namespace tinyxml2;

// ── Helpers ──────────────────────────────────────────────────────────────────

static Vec3 parseVec3(const char* s, Vec3 def = {0,0,0}) {
    if(!s) return def;
    float a=0, b=0, c=0;
    std::sscanf(s, "%f %f %f", &a, &b, &c);
    return {a, b, c};
}

static Quaternion parseQuat(const char* s) {
    // MJCF quat = "w x y z"
    if(!s) return Quaternion::identity();
    float w=1, x=0, y=0, z=0;
    std::sscanf(s, "%f %f %f %f", &w, &x, &y, &z);
    return Quaternion{w, x, y, z}.normalized();
}

static Quaternion parseEuler(const char* s, const char* seq) {
    // MJCF euler: values in radians, order given by eulerseq compiler attribute
    // Default eulerseq = "xyz"
    if(!s) return Quaternion::identity();
    float a=0, b=0, c=0;
    std::sscanf(s, "%f %f %f", &a, &b, &c);
    // Always treat as XYZ intrinsic regardless of seq for now
    (void)seq;
    return Quaternion::fromEulerXYZ(a, b, c);
}

static Quaternion parseRotation(const XMLElement* body) {
    if(body->Attribute("quat"))
        return parseQuat(body->Attribute("quat"));
    if(body->Attribute("euler"))
        return parseEuler(body->Attribute("euler"), "xyz");
    if(body->Attribute("axisangle")) {
        const char* s = body->Attribute("axisangle");
        float x=0,y=0,z=1,a=0;
        std::sscanf(s, "%f %f %f %f", &x, &y, &z, &a);
        return Quaternion::fromAxisAngle({x,y,z}, a);
    }
    return Quaternion::identity();
}

static JointType parseJointType(const char* s) {
    if(!s) return JointType::Fixed;
    if(std::strcmp(s, "hinge") == 0)  return JointType::Revolute;
    if(std::strcmp(s, "slide") == 0)  return JointType::Prismatic;
    if(std::strcmp(s, "free")  == 0)  return JointType::Free;
    if(std::strcmp(s, "ball")  == 0)  return JointType::Ball;
    return JointType::Fixed;
}

// ── Recursive body parser ────────────────────────────────────────────────────

static void parseBody(const XMLElement* bodyElem,
                      SceneNode*        parent,
                      RobotModel&       model,
                      const std::unordered_map<std::string,std::string>& meshMap)
{
    auto node = std::make_unique<SceneNode>();
    node->name     = bodyElem->Attribute("name") ? bodyElem->Attribute("name") : "";
    node->localPos = parseVec3(bodyElem->Attribute("pos"));
    node->localRot = parseRotation(bodyElem);

    // Parse first joint child (MJCF bodies typically have at most one joint)
    for(const XMLElement* jElem = bodyElem->FirstChildElement("joint");
        jElem; jElem = jElem->NextSiblingElement("joint"))
    {
        Joint j;
        j.name  = jElem->Attribute("name") ? jElem->Attribute("name") : "";
        j.type  = parseJointType(jElem->Attribute("type"));
        j.axis  = parseVec3(jElem->Attribute("axis"), {0,0,1}).normalized();

        // Joint limits
        const char* range = jElem->Attribute("range");
        if(range) {
            float lo=-kPi, hi=kPi;
            std::sscanf(range, "%f %f", &lo, &hi);
            j.limitLo = lo;
            j.limitHi = hi;
        }
        node->joint = j;
        break; // only first joint
    }

    // Parse freejoint shorthand
    const XMLElement* fj = bodyElem->FirstChildElement("freejoint");
    if(fj && node->joint.type == JointType::Fixed) {
        Joint j;
        j.name = fj->Attribute("name") ? fj->Attribute("name") : (node->name + "_free");
        j.type = JointType::Free;
        node->joint = j;
    }

    // Parse first mesh geom
    for(const XMLElement* gElem = bodyElem->FirstChildElement("geom");
        gElem; gElem = gElem->NextSiblingElement("geom"))
    {
        const char* type = gElem->Attribute("type");
        if(type && std::strcmp(type, "mesh") != 0) continue;

        const char* meshName = gElem->Attribute("mesh");
        if(!meshName) continue;

        auto it = meshMap.find(meshName);
        if(it != meshMap.end()) {
            // Assign mesh index (add path if new)
            auto pathIt = std::find(model.meshPaths.begin(),
                                    model.meshPaths.end(), it->second);
            if(pathIt == model.meshPaths.end()) {
                node->meshIndex = static_cast<int>(model.meshPaths.size());
                model.meshPaths.push_back(it->second);
            } else {
                node->meshIndex = static_cast<int>(
                    pathIt - model.meshPaths.begin());
            }
        }
        break; // first mesh geom only
    }

    // Recurse into child bodies
    SceneNode* rawNode = node.get();
    parent->addChild(std::move(node));

    for(const XMLElement* child = bodyElem->FirstChildElement("body");
        child; child = child->NextSiblingElement("body"))
    {
        parseBody(child, rawNode, model, meshMap);
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

std::unique_ptr<RobotModel> MJCFParser::parse(const std::string& xmlPath,
                                               const std::string& baseDir)
{
    XMLDocument doc;
    if(doc.LoadFile(xmlPath.c_str()) != XML_SUCCESS)
        throw std::runtime_error("MJCFParser: cannot load " + xmlPath);

    const XMLElement* root = doc.FirstChildElement("mujoco");
    if(!root)
        throw std::runtime_error("MJCFParser: no <mujoco> root element");

    std::string base = baseDir.empty()
        ? fs::path(xmlPath).parent_path().string()
        : baseDir;

    // ── 1. Build mesh name → absolute path map ──────────────────────────────
    std::unordered_map<std::string,std::string> meshMap;

    const XMLElement* assets = root->FirstChildElement("asset");
    if(assets) {
        for(const XMLElement* m = assets->FirstChildElement("mesh");
            m; m = m->NextSiblingElement("mesh"))
        {
            const char* name = m->Attribute("name");
            const char* file = m->Attribute("file");
            if(name && file) {
                fs::path p = fs::path(base) / file;
                meshMap[name] = p.string();
            }
        }
    }

    // ── 2. Parse worldbody ──────────────────────────────────────────────────
    auto model = std::make_unique<RobotModel>();

    // Synthetic world root (no joint, no mesh)
    model->root = std::make_unique<SceneNode>();
    model->root->name = "__world__";

    const XMLElement* wb = root->FirstChildElement("worldbody");
    if(wb) {
        for(const XMLElement* body = wb->FirstChildElement("body");
            body; body = body->NextSiblingElement("body"))
        {
            parseBody(body, model->root.get(), *model, meshMap);
        }
    }

    model->buildJointMap();
    model->update(); // initial FK pass

    return model;
}
