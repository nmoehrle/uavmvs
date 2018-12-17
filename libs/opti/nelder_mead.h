/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <array>
#include <functional>
#include <algorithm>

#include "math/vector.h"

template <int N>
struct Simplex {
    std::array<math::Vector<float, N>, N + 1> verts;
};

template <int N>
void
shrink(Simplex<N> * simplex, math::Vector<float, N> towards) {
    for (int i = 0; i < N + 1; ++i) {
        math::Vector<float, N> & vert = simplex->verts[i];
        vert = towards + (vert - towards) * 0.5f;
    }
}

template <int N>
std::pair<std::size_t, float>
nelder_mead(Simplex<N> * simplex, std::function<float(math::Vector<float, N>)> const & func) {
    std::array<float, N + 1> values;

    std::array<math::Vector<float, N>, N + 1> & verts = simplex->verts;
    for (std::size_t k = 0; k < values.size(); ++k) {
        values[k] = func(verts[k]);
    }

    std::vector<int> ranks(values.size());
    std::iota(ranks.begin(), ranks.end(), 0);
    std::sort(ranks.begin(), ranks.end(),
        [&values] (std::size_t a, std::size_t b) {
            return values[a] < values[b];
        }
    );

    std::size_t best_idx = ranks.front();
    std::size_t lousy_idx = ranks[ranks.size() - 2];
    std::size_t worst_idx = ranks.back();

    float best_value = values[best_idx];
    float lousy_value = values[lousy_idx];
    float worst_value = values[worst_idx];

    math::Vector<float, N> hpc(0.0f);
    for (std::size_t k = 0; k < ranks.size() - 1; ++k) {
        hpc += verts[ranks[k]];
    }
    hpc /= N;

    math::Vector<float, N> refl = hpc + (hpc - verts[worst_idx]);

    float refl_value = func(refl);

    if (refl_value < best_value) {
        math::Vector<float, N> exp = refl + (refl - hpc);
        float exp_value = func(exp);

        if (exp_value < best_value) {
            /* Expansion */
            verts[worst_idx] = exp;
            return {worst_idx, exp_value};
        } else {
            /* Reflection */
            verts[worst_idx] = refl;
            return {worst_idx, refl_value};
        }
    } else {
        if (refl_value < worst_value) {
            if (refl_value < lousy_value) {
                /* Reflection */
                verts[worst_idx] = refl;
                return {best_idx, best_value};
            } else {
                /* Outside contraction */
                math::Vector<float, N> con = hpc + (hpc - verts[worst_idx]) * 0.5f;
                float con_value = func(con);

                if (con_value < worst_value) {
                    verts[worst_idx] = con;
                    if (con_value < best_value) {
                        return {worst_idx, con_value};
                    } else {
                        return {best_idx, best_value};
                    }
                } else {
                    /* Shrink */
                    shrink(simplex, verts[best_idx]);
                    return {best_idx, best_value};
                }
            }
        } else {
            /* Inside contraction */
            math::Vector<float, N> con = hpc - (hpc - verts[worst_idx]) * 0.5f;
            float con_value = func(con);

            if (con_value < worst_value) {
                verts[worst_idx] = con;
                if (con_value < best_value) {
                    return {worst_idx, con_value};
                } else {
                    return {best_idx, best_value};
                }
            } else {
                /* Shrink */
                shrink(simplex, verts[best_idx]);
                return {best_idx, best_value};
            }
        }
    }
}

