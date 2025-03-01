#version 450

#include "tone_mapping.glsl"

layout(location=0) in vec3 position;
layout(location=4) in vec4 color;

layout(set=1, binding=0) uniform TranformBlock
{
    mat4 model;
    mat4 viewProj;
} transform;

layout(location=0) out vec4 pointColor;

void main() {
    gl_Position = transform.viewProj * transform.model * vec4(position, 1.0);
    gl_PointSize = 5.0;  // Fixed size or distance-based
    pointColor.rgb = uncharted2ToneMapping(color.rgb);
    pointColor.w = 1.0;
}
