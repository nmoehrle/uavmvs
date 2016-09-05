#version 330 core

in vec4 pos;
in vec3 normal;
in vec2 texuv;

out vec4 vpos;
out vec3 vnormal;
out vec2 vtexuv;

uniform mat4 modelviewmat;
uniform mat4 projmat;

void main(void)
{
    vpos = pos;
    vnormal = normal;
    vtexuv = texuv;

    gl_Position = projmat * (modelviewmat * pos);
}

