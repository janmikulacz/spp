/**
 * File:   generate_ccdt.cc
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/guards_ccdt.h"

#include <random>

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

using namespace trivis;

Guards guards::GuardsCCDT(
    const trivis::Trivis &vis,
    std::optional<double> vis_radius,
    std::optional<unsigned> n_max,
    std::optional<unsigned> random_seed,
    trivis::geom::FPolygons *guarded_triangles_out
) {
    Guards guards;
    if (n_max && n_max == 0) return guards;
    geom::FPolygons triangles;
    if (vis_radius) {
        mesh::TriangulateMapCCDT(vis.map(), vis_radius.value(), triangles);
    } else {
        triangles = vis.triangles();
    }
    auto indices = utils::Range(triangles.size());
    if (random_seed) {
        std::mt19937 rng(*random_seed);
        std::shuffle(indices.begin(), indices.end(), rng);
    }
    int cnt = 0;
    for (unsigned i: indices) {
        auto &triangle = triangles[i];
        geom::FPoint p;
        const auto &a = triangle[0];
        const auto &b = triangle[1];
        const auto &c = triangle[2];
        auto ab = b - a;
        auto ac = c - a;
        auto bc = c - b;
        double ab_len_sq = ab.NormSquared();
        double ac_len_sq = ac.NormSquared();
        double bc_len_sq = bc.NormSquared();
        std::vector<double> lengths = {ab_len_sq, ac_len_sq, bc_len_sq};
        std::vector<int> tri_ver_ids = {0, 1, 2};
        std::sort(tri_ver_ids.begin(), tri_ver_ids.end(), [lengths](int j, int k) { return lengths[j] < lengths[k]; });
        if (lengths[tri_ver_ids[0]] + lengths[tri_ver_ids[1]] > lengths[tri_ver_ids[2]]) {
            // acute triangle: p is the circumcenter
            double d = 2.0 * (ab.x * ac.y - ab.y * ac.x);
            double ux = (ac.y * ab_len_sq - ab.y * ac_len_sq);
            double uy = (ab.x * ac_len_sq - ac.x * ab_len_sq);
            p = geom::MakePoint(ux, uy) / d + a;
        } else {
            // right or obtuse triangle: p is the midpoint of the longest triangle edge
            if (tri_ver_ids[2] == 0) {
                p = (a + b) / 2.0;
            } else if (tri_ver_ids[2] == 1) {
                p = (a + c) / 2.0;
            } else {
                p = (b + c) / 2.0;
            }
        }
        guards.push_back({.point = p});
        if (guarded_triangles_out) {
            guarded_triangles_out->push_back(std::move(triangle));
        }
        if (n_max && ++cnt >= n_max) {
            break;
        }
    }
    return guards;
}
