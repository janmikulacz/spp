/**
 * File:   evaluate_vis.cc
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/statistics/evaluate_vis.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::statistics;

using namespace trivis;

using Clock = trivis::utils::SimpleClock;

std::vector<VisibilityRegionComputationMetrics> statistics::EvaluateVisibilityRegionComputations(
    const trivis::Trivis &vis,
    const std::optional<double> &vis_radius,
    const std::optional<trivis::geom::FPoints> &queries
) {
    int n = static_cast<int>(queries ? queries->size() : vis.mesh().vertices.size());
    std::vector<VisibilityRegionComputationMetrics> metrics(n);
    for (int i = 0; i < n; ++i) {
        auto &met = metrics[i];
        AbstractVisibilityRegion abs_reg;
        Trivis::ExpansionStats vis_stat;
        Clock clock;
        if (queries) {
            clock.Restart();
            const auto &q = (*queries)[i];
            auto pl = vis.LocatePoint(q);
            double time_buck = clock.TimeInSeconds();
            if (!pl) {
                continue;
            }
            met.time_bucketing = time_buck;
            met.tri_found = true;
            { // abs_reg_opt scope
                clock.Restart();
                abs_reg = vis.VisibilityRegion(q, pl.value(), vis_radius, &vis_stat);
                met.time_tea = clock.TimeInSeconds();
            }
        } else {
            met.time_bucketing = 0.0;
            met.tri_found = true;
            clock.Restart();
            abs_reg = vis.VisibilityRegion(i, vis_radius, &vis_stat);
            met.time_tea = clock.TimeInSeconds();
        }
        met.n_expansions = vis_stat.num_expansions;
        met.max_recursion_depth = vis_stat.max_recursion_depth;
        clock.Restart();
        auto reg = vis.ToRadialVisibilityRegion(abs_reg);
        met.time_intersections = clock.TimeInSeconds();
        int n_ver_before_antennas = static_cast<int>(reg.vertices.size());
        clock.Restart();
        RemoveAntennas(reg);
        met.time_antennas = clock.TimeInSeconds();
        int n_ver = static_cast<int>(reg.vertices.size());
        met.n_vertices = n_ver;
        met.had_antennas = n_ver != n_ver_before_antennas;
        if (vis_radius.has_value()) {
            clock.Restart();
            reg.IntersectWithCircleCenteredAtSeed(vis_radius.value());
            met.time_circ_intersect = clock.TimeInSeconds();
        }
        bool valid = IsValid(reg);
        met.is_valid = valid;
        if (!valid) {
            continue;
        }
        met.area = reg.Area();
        int n_minus_1 = n_ver - 1;
        for (int j_prev = n_minus_1, j = 0; j < reg.vertices.size(); j_prev = j++) {
            const auto &p_prev = reg.vertices[j_prev].point;
            const auto &p = reg.vertices[j].point;
            if (reg.vertices[j].edge_flag >= 0) {
                // obstacle edge
                met.obst_edges_total_len += p.DistanceTo(p_prev);
            } else if (reg.vertices[j].edge_flag == -1) {
                // free edge
                met.free_edges_total_len += p.DistanceTo(p_prev);
            } else if (reg.vertices[j].edge_flag == -2) {
                // free arc
                auto a = reg.seed - p_prev;
                auto b = reg.seed - p;
                auto angle = std::acos((a.x * b.x + a.y * b.y) / (a.Norm() * b.Norm()));
                assert(vis_radius.has_value());
                met.free_edges_total_len += angle * reg.radius.value();
            }
        }
        met.time_total = met.time_bucketing + met.time_tea + met.time_intersections + met.time_antennas + met.time_circ_intersect;
    }
    return metrics;
}
