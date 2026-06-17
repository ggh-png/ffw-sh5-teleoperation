#version 430 core

in vec3 vFragPos;
in vec3 vNormal;

uniform vec3 uObjectColor;
uniform vec3 uLightPos;
uniform vec3 uViewPos;

out vec4 FragColor;

void main() {
    // Ambient
    vec3 ambient = 0.18 * uObjectColor;

    // Diffuse
    vec3 norm     = normalize(vNormal);
    vec3 lightDir = normalize(uLightPos - vFragPos);
    float diff    = max(dot(norm, lightDir), 0.0);
    vec3 diffuse  = diff * uObjectColor;

    // Specular (Blinn-Phong)
    vec3 viewDir  = normalize(uViewPos - vFragPos);
    vec3 halfDir  = normalize(lightDir + viewDir);
    float spec    = pow(max(dot(norm, halfDir), 0.0), 64.0);
    vec3 specular = 0.4 * spec * vec3(1.0);

    FragColor = vec4(ambient + diffuse + specular, 1.0);
}
