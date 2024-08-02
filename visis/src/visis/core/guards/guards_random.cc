/**
 * File:   generate_random.cc
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/guards_random.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

Guards guards::GuardsRandom(
    const trivis::Trivis &vis,
    unsigned n_max,
    unsigned random_seed
) {
    Guards guards;
    if (n_max < 1) return guards;
    std::mt19937 rng(random_seed);
    for (int i = 0; i < n_max; ++i) {
        guards.push_back({.point = trivis::utils::UniformRandomPointInRandomTriangle(vis.triangles(), rng)});
    }
    return guards;
}
