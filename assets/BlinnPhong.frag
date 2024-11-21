#version 450

layout(location=0) in vec4 fragWorldPos;
layout(location=1) in vec3 fragNormal;

// Light structure
struct DirectionalLight {
    vec3 direction;
    vec3 color;
    float intensity;
};

// set=3 is required, see SDL3's documentation for SDL_CreateGPUShader
// https://wiki.libsdl.org/SDL3/SDL_CreateGPUShader
layout (set=3, binding=0) uniform Material {
    // material diffuse color
    vec4 baseColor;
    float metalness;
    float roughness;
    float ao;
} material;

layout(set=3, binding=1) uniform LightBlock {
    DirectionalLight light;
};

layout(location=0) out vec4 fragColor;

void main() {
    vec3 lightDir = normalize(-light.direction);
    vec3 normal = normalize(fragNormal);

    if (!gl_FrontFacing) {
        // Flip normal for back faces
        normal = -normal;
    }
    vec3 lightCol = light.intensity * light.color;
    // ambient color
    vec3 ambientColor = vec3(0.05, 0.05, 0.05);
    // lambertian diffuse term
    float diffRad = max(0., dot(lightDir, normal));
    vec3 diffuse = diffRad * lightCol;

    // specular term
    vec3 camDir = normalize(-fragWorldPos.xyz / fragWorldPos.w);
    vec3 h = normalize(lightDir + camDir);
    float spec = max(0., dot(h, normal));
    float shininess = 80;
    spec = pow(spec, shininess);
    vec3 specular = vec3(spec);

    fragColor.rgb = ambientColor;
    fragColor.rgb += material.baseColor.rgb * diffuse;
    fragColor.rgb += specular;
    fragColor = clamp(fragColor, 0., 1.);
}
