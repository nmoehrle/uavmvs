#version 330 core

in vec3 gnormal;
in vec2 gtexuv;

layout(location=0) out vec4 fcolor;

uniform mat4 modelviewmat;
uniform mat4 projmat;

uniform vec3 light = vec3(0.0, 0.0, -1.0);

uniform sampler2D tex;

void main(void)
{
    /* Assign material albedo. */
    vec4 albedo = texture2D(tex, gtexuv);

    /* Compute shading from normal and lights. */
    vec3 normal = gnormal;
    float lf = dot(gnormal, normalize(light));
    fcolor = albedo * max(lf, 0.0) * 0.6 + albedo * 0.4;
}

