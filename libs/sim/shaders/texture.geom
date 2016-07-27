#version 330 core

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in vec4 vpos[];
in vec3 vnormal[];
in vec2 vtexuv[];

out vec3 gnormal;
out vec2 gtexuv;

void main(void)
{
    vec3 v01 = vpos[1].xyz - vpos[0].xyz;
    vec3 v02 = vpos[2].xyz - vpos[0].xyz;
    vec3 normal = normalize(cross(v02, v01));

    for (int i = 0; i < 3; ++i) {
        gnormal = normal;
        gtexuv = vtexuv[i];
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}
