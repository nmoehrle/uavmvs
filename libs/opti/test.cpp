/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include "nelder_mead.h"

#define SQR(x) ((x) * (x))

float sqr(float x) {
    return x * x;
}

int main(int argc, char * argv[]) {
    std::function<float(math::Vector<float, 2>)> f = [] (math::Vec2f x) -> float {
        return SQR(1.0f - x[0]) + 100.0f * sqr(x[1] - SQR(x[0]));
    };

    Simplex<2> simplex;
    simplex.verts[0] = math::Vec2f(-1.3f, -1.3f);
    simplex.verts[1] = math::Vec2f(-1.2f, -1.2f);
    simplex.verts[2] = math::Vec2f(-1.2f, -1.4f);

    for (int i = 0; i < 50; ++i) {
        std::size_t idx;
        float value;
        std::tie(idx, value) = nelder_mead(&simplex, f);
        std::cout << simplex.verts[idx] << std::endl;
        std::cerr << value << std::endl;
    }
}
