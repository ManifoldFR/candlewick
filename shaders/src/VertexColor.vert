#version 450

layout(location=0) in vec3 position;
layout(location=4) in vec4 color;

layout(location=0) out vec4 interpColor;

// set=1 is required, for some god forsaken reason
// i am not that smart
layout(set=1, binding=0) uniform UniformBlock
{
    mat4 viewProj;
};

void main() {

    gl_Position = viewProj * vec4(position, 1.0);

    interpColor = color;
}
