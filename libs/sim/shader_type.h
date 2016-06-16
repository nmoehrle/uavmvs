#ifndef SIM_SHADERTYPE_HEADER
#define SIM_SHADERTYPE_HEADER

enum ShaderType {
    VCOLOR = 0,
    SURFACE = 1,
    TEXTURE = 2,
    LINES = 3
};

static const char * shader_names[] = {"vcolor", "surface", "texture", "lines"};

#endif /* SIM_SHADERTYPE_HEADER */
