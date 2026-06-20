#version 430 core

in vec3 vFragPos;
in vec3 vNormal;
in vec4 vFragPosLight;
in vec2 vTexCoord;

uniform vec3       uObjectColor;
uniform vec3       uLightPos;
uniform vec3       uViewPos;
uniform sampler2D  uShadowMap;    // unit 0
uniform sampler2D  uTexDiffuse;   // unit 1  (only used when uUseTexture != 0)
uniform int        uUseTexture;   // 0 = use uObjectColor, 1 = sample uTexDiffuse

out vec4 FragColor;

float computeShadow(vec4 fragPosLS) {
    vec3 proj = fragPosLS.xyz / fragPosLS.w;
    proj = proj * 0.5 + 0.5;
    if(proj.z > 1.0) return 0.0;
    float currentDepth = proj.z;
    vec3 norm     = normalize(vNormal);
    vec3 lightDir = normalize(uLightPos - vFragPos);
    float bias    = max(0.008 * (1.0 - dot(norm, lightDir)), 0.002);
    float shadow   = 0.0;
    vec2 texelSize = 1.0 / textureSize(uShadowMap, 0);
    for(int x = -1; x <= 1; ++x)
        for(int y = -1; y <= 1; ++y) {
            float pcfDepth = texture(uShadowMap, proj.xy + vec2(x,y)*texelSize).r;
            shadow += (currentDepth - bias > pcfDepth) ? 1.0 : 0.0;
        }
    return shadow / 9.0;
}

void main() {
    vec3 baseColor = (uUseTexture != 0)
        ? texture(uTexDiffuse, vTexCoord).rgb
        : uObjectColor;

    vec3 ambient = 0.20 * baseColor;

    vec3 norm     = normalize(vNormal);
    vec3 lightDir = normalize(uLightPos - vFragPos);
    float diff    = max(dot(norm, lightDir), 0.0);
    vec3 diffuse  = diff * baseColor;

    vec3 viewDir  = normalize(uViewPos - vFragPos);
    vec3 halfDir  = normalize(lightDir + viewDir);
    float spec    = pow(max(dot(norm, halfDir), 0.0), 64.0);
    vec3 specular = 0.35 * spec * vec3(1.0);

    float shadow = computeShadow(vFragPosLight);
    vec3 lit     = ambient + (1.0 - shadow * 0.75) * (diffuse + specular);

    FragColor = vec4(lit, 1.0);
}
