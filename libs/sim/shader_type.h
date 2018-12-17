/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

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
