#include "SceneLoader.hpp"
#include <tinyxml2.h>
#include <cstdio>
#include <cstring>
#include <sstream>

// ── helper: parse "a b c ..." into floats ─────────────────────────────────────

static int parseFloats(const char* s, float* out, int maxN) {
    if(!s) return 0;
    std::istringstream ss(s);
    int n = 0;
    while(n < maxN && (ss >> out[n])) ++n;
    return n;
}

// ── SceneLoader::load ─────────────────────────────────────────────────────────

std::vector<ObjectDesc> SceneLoader::load(const std::string& xmlPath) {
    std::vector<ObjectDesc> result;

    tinyxml2::XMLDocument doc;
    if(doc.LoadFile(xmlPath.c_str()) != tinyxml2::XML_SUCCESS) {
        fprintf(stderr, "[SceneLoader] Cannot load '%s': %s\n",
                xmlPath.c_str(), doc.ErrorStr());
        return result;
    }

    auto* sceneEl = doc.FirstChildElement("scene");
    if(!sceneEl) {
        fprintf(stderr, "[SceneLoader] No <scene> root in '%s'\n", xmlPath.c_str());
        return result;
    }

    for(auto* bodyEl = sceneEl->FirstChildElement("body");
             bodyEl;
             bodyEl = bodyEl->NextSiblingElement("body")) {

        ObjectDesc desc;
        desc.name = bodyEl->Attribute("name") ? bodyEl->Attribute("name") : "";

        // Body position
        float pos3[3] = {};
        parseFloats(bodyEl->Attribute("pos"), pos3, 3);
        desc.pos = {pos3[0], pos3[1], pos3[2]};

        // First <geom> child carries shape + physics attributes
        auto* geomEl = bodyEl->FirstChildElement("geom");
        if(!geomEl) continue;

        const char* typeStr = geomEl->Attribute("type");
        if(!typeStr) continue;

        if(strcmp(typeStr, "box") == 0) {
            desc.type = ObjectDesc::Type::Box;
            float sz[3] = {};
            parseFloats(geomEl->Attribute("size"), sz, 3);
            desc.halfExtents = {sz[0], sz[1], sz[2]};
        } else if(strcmp(typeStr, "cylinder") == 0) {
            desc.type = ObjectDesc::Type::Cylinder;
            float sz[2] = {};
            parseFloats(geomEl->Attribute("size"), sz, 2);
            desc.radius     = sz[0];
            desc.halfHeight = sz[1];
        } else if(strcmp(typeStr, "mesh") == 0) {
            desc.type = ObjectDesc::Type::Mesh;
            const char* file = geomEl->Attribute("file");
            if(file) {
                // Resolve file path relative to the XML directory
                std::string xmlDir = xmlPath;
                auto slash = xmlDir.find_last_of("/\\");
                if(slash != std::string::npos) xmlDir.resize(slash + 1); else xmlDir = "./";
                desc.meshFile = xmlDir + file;
            }
            geomEl->QueryFloatAttribute("scale", &desc.meshScale);
        } else {
            fprintf(stderr, "[SceneLoader] Unknown geom type '%s' — skipped\n", typeStr);
            continue;
        }

        geomEl->QueryFloatAttribute("mass",     &desc.mass);
        geomEl->QueryFloatAttribute("friction", &desc.friction);

        float rgba[4] = {0.8f, 0.8f, 0.8f, 1.f};
        parseFloats(geomEl->Attribute("rgba"), rgba, 4);
        desc.color = {rgba[0], rgba[1], rgba[2]};

        result.push_back(std::move(desc));
        fprintf(stderr, "[SceneLoader] Loaded '%s' (%s, mass=%.3f)\n",
                result.back().name.c_str(),
                typeStr, result.back().mass);
    }

    fprintf(stderr, "[SceneLoader] Loaded %zu objects from '%s'\n",
            result.size(), xmlPath.c_str());
    return result;
}
