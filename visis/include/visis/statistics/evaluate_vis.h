/**
 * File:   evaluate_vis.h
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_STATISTICS_EVALUATE_VIS_H_
#define VISIS_STATISTICS_EVALUATE_VIS_H_

#include "trivis/trivis.h"

namespace visis::statistics {

struct VisibilityRegionComputationMetrics {
    bool tri_found = false;
    double time_bucketing = 0.0;
    double time_tea = 0.0;
    int n_expansions = 0;
    int max_recursion_depth = 0;
    double time_intersections = 0.0;
    double time_antennas = 0.0;
    int n_vertices = 0;
    bool had_antennas = false;
    double time_circ_intersect = 0.0;
    bool is_valid = false;
    double area = 0.0;
    double free_edges_total_len = 0.0;
    double obst_edges_total_len = 0.0;
    double time_total = 0.0;
};

std::vector<VisibilityRegionComputationMetrics> EvaluateVisibilityRegionComputations(
    const trivis::Trivis &vis,
    const std::optional<double> &vis_radius,
    const std::optional<trivis::geom::FPoints> &queries = std::nullopt
);

}

#endif //VISIS_STATISTICS_EVALUATE_VIS_H_
