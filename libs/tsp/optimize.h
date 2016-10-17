#ifndef TSP_OPTIMIZE_HEADER
#define TSP_OPTIMIZE_HEADER

#include <vector>

#include "math/vector.h"

#include "defines.h"

TSP_NAMESPACE_BEGIN


float twoopt(std::vector<uint> * ids, std::vector<float> const & sqdists);

template <int N>
float optimize(std::vector<uint> * ids, std::vector<math::Vector<float, N> > const & verts,
    int iters = 1000)
{
    std::vector<float> sqdists((ids->size() - 1) * ids->size() / 2);
    for (std::size_t idx = 0, i = 1; i < ids->size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            sqdists[idx++] = (verts[j] - verts[i]).square_norm();
        }
    }

    auto sqdist = [&ids, &sqdists] (std::size_t i, std::size_t j) {
        std::tie(j, i) = std::minmax(ids->at(i), ids->at(j));
        return sqdists[((i - 1) * i) / 2 + j];
    };

    float length = std::sqrt(sqdist(ids->size() - 1, 0));
    for (std::size_t i = 0; i < ids->size() - 1; ++i) {
        length += std::sqrt(sqdist(i, i + 1));
    }

    #pragma omp parallel
    {
        std::mt19937 gen;
        std::vector<uint> nids(ids->size());
        #pragma omp for
        for (int i = 0; i < iters; ++i) {
            gen.seed(i);
            std::iota(nids.begin(), nids.end(), 0);
            std::shuffle(nids.begin(), nids.end(), gen);
            float nlength = twoopt(&nids, sqdists);
            #pragma omp critical
            if (nlength < length) {
                std::swap(*ids, nids);
                length = nlength;
            }
        }
    }

    return length;
}

float twoopt(std::vector<uint> * ids, std::vector<float> const & sqdists) {
    auto sqdist = [&ids, &sqdists] (std::size_t i, std::size_t j) {
        std::tie(j, i) = std::minmax(ids->at(i), ids->at(j));
        return sqdists[((i - 1) * i) / 2 + j];
    };

    while (true) {
        float best = 0.0f;
        std::pair<uint, uint> move;
        for (std::size_t i = 0; i < ids->size() - 2; ++i) {
            float sdii1 = sqdist(i, i + 1);
            for (std::size_t j = i + 2; j < ids->size() - 1; ++j) {
                float change = sqdist(i, j) + sqdist(i + 1, j + 1)
                    - sdii1 - sqdist(j, j + 1);
                if (change < best) {
                    best = change;
                    move = std::make_pair(i, j);
                }
            }

            /* Unrolled wraparound */
            {
                std::size_t j = ids->size() - 1;
                float change = sqdist(i, j) + sqdist(i + 1, 0)
                    - sdii1 - sqdist(j, 0);
                if (change < best) {
                    best = change;
                    move = std::make_pair(i, j);
                }
            }
        }

        if (best >= -1e-5f) break;

        for (std::size_t i = move.first + 1, j = move.second; i < j; ++i, --j) {
            std::swap(ids->at(i), ids->at(j));
        }
    }

    float length = std::sqrt(sqdist(ids->size() - 1, 0));
    for (std::size_t i = 0; i < ids->size() - 1; ++i) {
        length += std::sqrt(sqdist(i, i + 1));
    }
    return length;
}

TSP_NAMESPACE_END

#endif /* TSP_OPTIMIZE_HEADER */
