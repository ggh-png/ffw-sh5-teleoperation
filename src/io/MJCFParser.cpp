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

// ── Primitive parsers ─────────────────────────────────────────────────────────

static Vec3 parseVec3(const char* s, Vec3 def = {0,0,0}) {
    if(!s) return def;
    float a=0, b=0, c=0;
    std::sscanf(s, "%f %f %f", &a, &b, &c);
    return {a, b, c};
}

static Quaternion parseQuat(const char* s) {
    if(!s) return Quaternion::identity();
    float w=1, x=0, y=0, z=0;
    std::sscanf(s, "%f %f %f %f", &w, &x, &y, &z);
    return Quaternion{w, x, y, z}.normalized();
}

static Quaternion parseEuler(const char* s) {
    if(!s) return Quaternion::identity();
    float a=0, b=0, c=0;
    std::sscanf(s, "%f %f %f", &a, &b, &c);
    return Quaternion::fromEulerXYZ(a, b, c);
}

static Quaternion parseRotation(const XMLElement* body) {
    if(body->Attribute("quat"))
        return parseQuat(body->Attribute("quat"));
    if(body->Attribute("euler"))
        return parseEuler(body->Attribute("euler"));
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

static void parseRange(const char* s, float& lo, float& hi) {
    if(!s) return;
    std::sscanf(s, "%f %f", &lo, &hi);
}

// ── Default-class system ──────────────────────────────────────────────────────
// MJCF <default class="..."> provides inherited property values for joints,
// geoms, and actuators.  Nested classes inherit from their parent (CSS cascade).

struct JointDefaults {
    bool      hasType    = false; JointType type       = JointType::Fixed;
    bool      hasAxis    = false; Vec3      axis       = {0,0,1};
    bool      hasRange   = false; float     lo = -kPi, hi = kPi;
    // MuJoCo joint dynamics
    bool      hasDamping = false; float     damping    = 0.f;
    bool      hasFriction= false; float     frictionLoss = 0.f;
    bool      hasArmature= false; float     armature   = 0.f;
    // Actuator (position/velocity controller)
    bool      hasKp      = false; float     kp         = 0.f;
    bool      hasForce   = false; float     forceMin   = -1e9f, forceMax = 1e9f;
    bool      isVelCtrl  = false; // true when actuator type is <velocity>
};

struct GeomDefaults {
    bool hasGroup   = false; int   group    = 0;
    bool hasColor   = false; Vec3  color    = {0.7f,0.7f,0.75f};
    bool hasFriction= false; float friction = 0.7f;  // sliding friction coeff
};

struct DefaultClass {
    std::string   parent; // empty = root default
    JointDefaults joint;
    GeomDefaults  geom;
};

using DefaultMap = std::unordered_map<std::string, DefaultClass>;

static void parseDefaultElem(const XMLElement* elem,
                              const std::string& parentCls,
                              DefaultMap& defs)
{
    const char* clsAttr = elem->Attribute("class");
    std::string cls = clsAttr ? clsAttr : "";

    DefaultClass dc;
    dc.parent = parentCls;

    // ── Joint sub-element ─────────────────────────────────────────────────
    const XMLElement* jElem = elem->FirstChildElement("joint");
    if(jElem) {
        if(jElem->Attribute("type"))  {
            dc.joint.hasType = true;
            dc.joint.type = parseJointType(jElem->Attribute("type"));
        }
        if(jElem->Attribute("axis"))  {
            dc.joint.hasAxis = true;
            dc.joint.axis = parseVec3(jElem->Attribute("axis")).normalized();
        }
        if(jElem->Attribute("range")) {
            dc.joint.hasRange = true;
            parseRange(jElem->Attribute("range"), dc.joint.lo, dc.joint.hi);
        }
        if(jElem->Attribute("damping")) {
            dc.joint.hasDamping = true;
            jElem->QueryFloatAttribute("damping", &dc.joint.damping);
        }
        if(jElem->Attribute("frictionloss")) {
            dc.joint.hasFriction = true;
            jElem->QueryFloatAttribute("frictionloss", &dc.joint.frictionLoss);
        }
        if(jElem->Attribute("armature")) {
            dc.joint.hasArmature = true;
            jElem->QueryFloatAttribute("armature", &dc.joint.armature);
        }
    }

    // ── Actuator sub-element (<position>, <velocity>, or <motor>) ─────────
    // Velocity-controlled joints (wheel drives) are flagged with isVelCtrl so
    // JointDynamics skips gravity-sag computation for them.
    for(const char* tag : {"position", "velocity", "motor"}) {
        const XMLElement* aElem = elem->FirstChildElement(tag);
        if(!aElem) continue;
        if(std::strcmp(tag, "velocity") == 0) dc.joint.isVelCtrl = true;
        if(aElem->Attribute("kp")) {
            dc.joint.hasKp = true;
            aElem->QueryFloatAttribute("kp", &dc.joint.kp);
        }
        if(aElem->Attribute("kv")) {
            dc.joint.hasKp  = true;
            dc.joint.isVelCtrl = true;
            aElem->QueryFloatAttribute("kv", &dc.joint.kp);
        }
        if(aElem->Attribute("forcerange")) {
            dc.joint.hasForce = true;
            std::sscanf(aElem->Attribute("forcerange"),
                        "%f %f", &dc.joint.forceMin, &dc.joint.forceMax);
        }
        break;
    }

    // ── Geom sub-element ──────────────────────────────────────────────────
    const XMLElement* gElem = elem->FirstChildElement("geom");
    if(gElem) {
        if(gElem->Attribute("group")) {
            dc.geom.hasGroup = true;
            gElem->QueryIntAttribute("group", &dc.geom.group);
        }
        if(gElem->Attribute("rgba")) {
            dc.geom.hasColor = true;
            float r=0.7f,g=0.7f,b=0.75f,a=1.f;
            std::sscanf(gElem->Attribute("rgba"), "%f %f %f %f", &r,&g,&b,&a);
            dc.geom.color = {r, g, b};
        }
        if(gElem->Attribute("friction")) {
            dc.geom.hasFriction = true;
            // MJCF friction="sliding torsional rolling" — only sliding matters for Bullet
            std::sscanf(gElem->Attribute("friction"), "%f", &dc.geom.friction);
        }
    }

    defs[cls] = dc;

    // Recurse into nested <default class="...">
    for(const XMLElement* child = elem->FirstChildElement("default");
        child; child = child->NextSiblingElement("default"))
    {
        parseDefaultElem(child, cls, defs);
    }
}

// Resolve a class name to its fully-merged JointDefaults (parent chain → child)
static JointDefaults resolveJointDef(const std::string& cls,
                                      const DefaultMap& defs)
{
    std::vector<const DefaultClass*> chain;
    std::string cur = cls;
    while(true) {
        auto it = defs.find(cur);
        if(it == defs.end()) break;
        chain.push_back(&it->second);
        if(it->second.parent.empty()) break;
        cur = it->second.parent;
    }
    JointDefaults result;
    for(auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const JointDefaults& d = (*it)->joint;
        if(d.hasType)    { result.hasType    = true; result.type       = d.type;       }
        if(d.hasAxis)    { result.hasAxis    = true; result.axis       = d.axis;       }
        if(d.hasRange)   { result.hasRange   = true; result.lo=d.lo; result.hi=d.hi;   }
        if(d.hasDamping) { result.hasDamping = true; result.damping    = d.damping;    }
        if(d.hasFriction){ result.hasFriction= true; result.frictionLoss=d.frictionLoss;}
        if(d.hasArmature){ result.hasArmature= true; result.armature   = d.armature;   }
        if(d.hasKp)      { result.hasKp      = true; result.kp         = d.kp;         }
        if(d.hasForce)   { result.hasForce   = true;
                           result.forceMin = d.forceMin; result.forceMax = d.forceMax; }
        if(d.isVelCtrl)    result.isVelCtrl  = true;
    }
    return result;
}

// Resolve geom defaults (group, color, friction)
static GeomDefaults resolveGeomDef(const std::string& cls,
                                    const DefaultMap& defs)
{
    std::vector<const DefaultClass*> chain;
    std::string cur = cls;
    while(true) {
        auto it = defs.find(cur);
        if(it == defs.end()) break;
        chain.push_back(&it->second);
        if(it->second.parent.empty()) break;
        cur = it->second.parent;
    }
    GeomDefaults result;
    for(auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const GeomDefaults& d = (*it)->geom;
        if(d.hasGroup)   { result.hasGroup   = true; result.group    = d.group;    }
        if(d.hasColor)   { result.hasColor   = true; result.color    = d.color;    }
        if(d.hasFriction){ result.hasFriction= true; result.friction = d.friction; }
    }
    return result;
}

// ── Recursive body parser ─────────────────────────────────────────────────────

static void parseBody(const XMLElement* bodyElem,
                      SceneNode*        parent,
                      RobotModel&       model,
                      const std::unordered_map<std::string,std::string>& meshMap,
                      const std::unordered_map<std::string,Vec3>& meshScaleMap,
                      const std::unordered_map<std::string,Vec3>& materialMap,
                      const DefaultMap& defs)
{
    auto node = std::make_unique<SceneNode>();
    node->name     = bodyElem->Attribute("name") ? bodyElem->Attribute("name") : "";
    node->localPos = parseVec3(bodyElem->Attribute("pos"));
    node->localRot = parseRotation(bodyElem);

    // ── <inertial> ────────────────────────────────────────────────────────
    const XMLElement* inEl = bodyElem->FirstChildElement("inertial");
    if(inEl) {
        node->inertial.valid = true;
        if(inEl->Attribute("mass"))
            inEl->QueryFloatAttribute("mass", &node->inertial.mass);
        if(inEl->Attribute("pos"))
            node->inertial.com = parseVec3(inEl->Attribute("pos"));
        if(inEl->Attribute("diaginertia")) {
            Vec3 di = parseVec3(inEl->Attribute("diaginertia"));
            node->inertial.diagInertia = di;
        }
        if(inEl->Attribute("quat"))
            node->inertial.frame = parseQuat(inEl->Attribute("quat"));
    }

    // ── Parse first joint child ───────────────────────────────────────────
    for(const XMLElement* jElem = bodyElem->FirstChildElement("joint");
        jElem; jElem = jElem->NextSiblingElement("joint"))
    {
        Joint j;
        j.name = jElem->Attribute("name") ? jElem->Attribute("name") : "";
        j.type = JointType::Revolute;

        // 1. Resolve defaults from class inheritance
        const char* cls = jElem->Attribute("class");
        if(cls) {
            JointDefaults d = resolveJointDef(cls, defs);
            if(d.hasType)    j.type       = d.type;
            if(d.hasAxis)    j.axis       = d.axis;
            if(d.hasRange)   { j.hasLimits=true; j.limitLo=d.lo; j.limitHi=d.hi; }
            if(d.hasDamping) j.damping    = d.damping;
            if(d.hasFriction)j.frictionLoss = d.frictionLoss;
            if(d.hasArmature)j.armature   = d.armature;
            if(d.hasKp)      j.kp         = d.kp;
            if(d.hasForce)   { j.forceMin = d.forceMin; j.forceMax = d.forceMax; }
            if(d.isVelCtrl)  j.isVelocityCtrl = true;
        }

        // 2. Explicit attributes override defaults
        if(jElem->Attribute("type"))
            j.type = parseJointType(jElem->Attribute("type"));
        if(jElem->Attribute("axis"))
            j.axis = parseVec3(jElem->Attribute("axis")).normalized();
        if(jElem->Attribute("range")) {
            j.hasLimits = true;
            parseRange(jElem->Attribute("range"), j.limitLo, j.limitHi);
        }
        if(jElem->Attribute("damping"))
            jElem->QueryFloatAttribute("damping", &j.damping);
        if(jElem->Attribute("frictionloss"))
            jElem->QueryFloatAttribute("frictionloss", &j.frictionLoss);
        if(jElem->Attribute("armature"))
            jElem->QueryFloatAttribute("armature", &j.armature);

        node->joint = j;
        break; // one joint per body
    }

    // Parse <freejoint> shorthand
    const XMLElement* fj = bodyElem->FirstChildElement("freejoint");
    if(fj && node->joint.type == JointType::Fixed) {
        Joint j;
        j.name = fj->Attribute("name") ? fj->Attribute("name") : (node->name + "_free");
        j.type = JointType::Free;
        node->joint = j;
    }

    // ── Parse visual mesh geom ────────────────────────────────────────────
    // Prefer group=2 (visual), skip group=3 (collision-only) geoms.
    for(const XMLElement* gElem = bodyElem->FirstChildElement("geom");
        gElem; gElem = gElem->NextSiblingElement("geom"))
    {
        const char* gCls = gElem->Attribute("class");
        GeomDefaults gd;
        if(gCls) gd = resolveGeomDef(gCls, defs);

        int group = gd.group;
        if(gElem->Attribute("group"))
            gElem->QueryIntAttribute("group", &group);
        if(group == 3) continue;
        if(gCls && std::strcmp(gCls, "collision") == 0) continue;

        const char* type = gElem->Attribute("type");
        if(type && std::strcmp(type, "mesh") != 0) continue;
        if(!type) {
            if(!gCls) continue;
        }

        const char* meshName = gElem->Attribute("mesh");
        if(!meshName) continue;

        const char* matName = gElem->Attribute("material");
        if(matName) {
            auto mit = materialMap.find(matName);
            if(mit != materialMap.end())
                node->color = mit->second;
        } else if(gElem->Attribute("rgba")) {
            float r=0.7f,g=0.7f,b=0.75f,a=1.f;
            std::sscanf(gElem->Attribute("rgba"), "%f %f %f %f",&r,&g,&b,&a);
            node->color = {r,g,b};
        }

        auto it = meshMap.find(meshName);
        if(it != meshMap.end()) {
            auto pathIt = std::find(model.meshPaths.begin(),
                                    model.meshPaths.end(), it->second);
            if(pathIt == model.meshPaths.end()) {
                node->meshIndex = static_cast<int>(model.meshPaths.size());
                model.meshPaths.push_back(it->second);
            } else {
                node->meshIndex = static_cast<int>(
                    pathIt - model.meshPaths.begin());
            }
            auto scaleIt = meshScaleMap.find(meshName);
            if(scaleIt != meshScaleMap.end())
                node->meshScale = scaleIt->second;
        }
        break;
    }

    // ── Parse first collision geom for friction + radius ──────────────────
    // Collision geoms (group=3) carry per-material friction and, for
    // cylinder/sphere types, the contact radius used to derive geometry
    // constants (e.g. kWheelRadius).
    for(const XMLElement* gElem = bodyElem->FirstChildElement("geom");
        gElem; gElem = gElem->NextSiblingElement("geom"))
    {
        const char* gCls = gElem->Attribute("class");
        GeomDefaults gd;
        if(gCls) gd = resolveGeomDef(gCls, defs);

        int group = gd.group;
        if(gElem->Attribute("group"))
            gElem->QueryIntAttribute("group", &group);
        // Accept explicit collision class or group=3
        bool isCollision = (group == 3) ||
                           (gCls && std::strcmp(gCls, "collision") == 0);
        if(!isCollision) continue;

        // Friction: inline attribute overrides default
        if(gElem->Attribute("friction")) {
            std::sscanf(gElem->Attribute("friction"), "%f", &node->geomFriction);
        } else if(gd.hasFriction) {
            node->geomFriction = gd.friction;
        }

        // Radius from cylinder or sphere geom
        const char* type = gElem->Attribute("type");
        if(type && (std::strcmp(type,"cylinder")==0 || std::strcmp(type,"sphere")==0)) {
            const char* sz = gElem->Attribute("size");
            if(sz) {
                float r = 0.f;
                std::sscanf(sz, "%f", &r);  // first value = radius for cyl/sphere
                if(r > 0.f) node->geomRadius = r;
            }
        }
        break; // first collision geom wins
    }

    // ── Recurse ───────────────────────────────────────────────────────────
    SceneNode* rawNode = node.get();
    parent->addChild(std::move(node));

    for(const XMLElement* child = bodyElem->FirstChildElement("body");
        child; child = child->NextSiblingElement("body"))
    {
        parseBody(child, rawNode, model, meshMap, meshScaleMap,
                  materialMap, defs);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

ParseResult MJCFParser::parse(const std::string& xmlPath,
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

    ParseResult result;

    // ── 0. <compiler> meshdir ─────────────────────────────────────────────
    fs::path meshBase = fs::path(base);
    const XMLElement* compiler = root->FirstChildElement("compiler");
    if(compiler) {
        const char* meshdir = compiler->Attribute("meshdir");
        if(meshdir && meshdir[0] != '\0')
            meshBase = fs::path(base) / meshdir;
    }

    // ── 1. <option> global physics settings ───────────────────────────────
    // MJCF uses Z-up; gravity default = (0, 0, -9.81).
    // We convert: Y-up gravity_y = MJCF gravity_z.
    const XMLElement* opt = root->FirstChildElement("option");
    if(opt) {
        const char* grav = opt->Attribute("gravity");
        if(grav) {
            float gx=0.f, gy=0.f, gz=-9.81f;
            std::sscanf(grav, "%f %f %f", &gx, &gy, &gz);
            result.physics.gravity = gz; // MJCF Z → Y-up Y
        }
        const char* ts = opt->Attribute("timestep");
        if(ts) std::sscanf(ts, "%f", &result.physics.timestep);

        float impr = 1.f;
        if(opt->QueryFloatAttribute("impratio", &impr) == XML_SUCCESS)
            result.physics.impratio = impr;
    }

    // ── 2. Parse <default> hierarchy ──────────────────────────────────────
    DefaultMap defs;
    const XMLElement* defaultRoot = root->FirstChildElement("default");
    if(defaultRoot) {
        parseDefaultElem(defaultRoot, "", defs);
    }

    // ── 3. Parse <asset>: meshes + materials ──────────────────────────────
    std::unordered_map<std::string,std::string> meshMap;
    std::unordered_map<std::string,Vec3>        meshScaleMap;
    std::unordered_map<std::string,Vec3>        materialMap;

    const XMLElement* assets = root->FirstChildElement("asset");
    if(assets) {
        for(const XMLElement* m = assets->FirstChildElement("mesh");
            m; m = m->NextSiblingElement("mesh"))
        {
            const char* name  = m->Attribute("name");
            const char* file  = m->Attribute("file");
            if(name && file) {
                meshMap[name] = (meshBase / file).string();
                const char* sc = m->Attribute("scale");
                if(sc) meshScaleMap[name] = parseVec3(sc, {1,1,1});
            }
        }
        for(const XMLElement* mat = assets->FirstChildElement("material");
            mat; mat = mat->NextSiblingElement("material"))
        {
            const char* name = mat->Attribute("name");
            const char* rgba = mat->Attribute("rgba");
            if(name && rgba) {
                float r=0.7f,g=0.7f,b=0.75f,a=1.f;
                std::sscanf(rgba, "%f %f %f %f", &r,&g,&b,&a);
                materialMap[name] = {r, g, b};
            }
        }
    }

    // ── 4. Parse worldbody ────────────────────────────────────────────────
    result.model = std::make_unique<RobotModel>();

    result.model->root = std::make_unique<SceneNode>();
    result.model->root->name = "__world__";
    // MuJoCo is Z-up; OpenGL renderer is Y-up.
    // Rotate world root -90° around X so MJCF Z→Y, MJCF Y→-Z.
    result.model->root->localRot = Quaternion::fromAxisAngle({1,0,0}, -kPi / 2.f);

    const XMLElement* wb = root->FirstChildElement("worldbody");
    if(wb) {
        for(const XMLElement* body = wb->FirstChildElement("body");
            body; body = body->NextSiblingElement("body"))
        {
            parseBody(body, result.model->root.get(), *result.model,
                      meshMap, meshScaleMap, materialMap, defs);
        }
    }

    result.model->buildJointMap();
    result.model->update();

    return result;
}
