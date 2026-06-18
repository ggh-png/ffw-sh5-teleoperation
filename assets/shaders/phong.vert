#version 430 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform mat4 uNormal;
uniform mat4 uLightSpace;

out vec3 vFragPos;
out vec3 vNormal;
out vec4 vFragPosLight;

void main() {
    vec4 worldPos  = uModel * vec4(aPos, 1.0);
    vFragPos       = worldPos.xyz;
    vNormal        = normalize(mat3(uNormal) * aNormal);
    vFragPosLight  = uLightSpace * worldPos;
    gl_Position    = uProj * uView * worldPos;
}
