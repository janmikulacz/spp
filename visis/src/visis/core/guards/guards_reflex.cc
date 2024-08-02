/**
 * File:   generate_reflex.cc
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/guards_reflex.h"

#include <random>

#include "visis/core/guards/reflex_vertex.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

using namespace trivis;

Guards guards::GuardsReflex(
    const trivis::Trivis &vis,
    double reflex_tolerance_deg,
    double dist_from_vertex,
    std::optional<unsigned> n_max,
    std::optional<unsigned> random_seed
) {
    Guards guards;
    if (n_max && n_max == 0) return guards;
    int cnt = 0;
    int n_vertices = static_cast<int>(vis.mesh().vertices.size());
    double reflex_tolerance = reflex_tolerance_deg > 0.0 ? reflex_tolerance_deg * M_PI / 180.0 : 0.0;
    auto indices = utils::Range(n_vertices);
    if (random_seed) {
        std::mt19937 rng(*random_seed);
        std::shuffle(indices.begin(), indices.end(), rng);
    }
    for (int i: indices) {
        auto neighbors = GetNeighborVertices(vis.mesh(), i);
        if (neighbors.size() != 2) {
            continue;
        }
        if (!IsReflex(vis.mesh(), neighbors[0], i, neighbors[1], reflex_tolerance)) {
            continue;
        }
        geom::FPoint p;
        if (dist_from_vertex > 0.0) {
            p = MovePointAwayFromVertex(vis, neighbors[0], i, neighbors[1], dist_from_vertex);
        } else {
            p = vis.mesh().point(i);
        }
        guards.push_back({.point = p, .is_anchor_0 = false, .is_anchor_n = false, .id_v = i});
        if (n_max && ++cnt >= n_max) {
            break;
        }
    }
    return guards;
}
