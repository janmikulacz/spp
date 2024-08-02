/**
 * File:   generate_ka.cc
 *
 * Date:   28.02.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/guards_ka.h"

#include <random>

#include "trivis/trivis.h"
#include "trivis/geom/intersections.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

using namespace trivis;

geom::FPoint guards::GuardWS(const geom::FPolygon &convex_polygon) {
    auto ws = geom::MakePoint(0.0, 0.0);
    double length_total = 0.0;
    int n_polygon = static_cast<int>(convex_polygon.size());
    for (int i_prev = n_polygon - 1, i = 0; i < n_polygon; i_prev = i++) {
        const auto &p1 = convex_polygon[i_prev];
        const auto &p2 = convex_polygon[i];
        double length = p1.DistanceTo(p2);
        length_total += length;
        ws = ws + (p1 + p2) * (length / 2.0);
    }
    ws = ws / length_total;
    return ws;
}

void AddGuardOrSplit(
    int depth,
    const geom::FPolygon &convex_polygon,
    const Trivis &vis,
    const visibility::VisibilityModel &vis_model,
    geom::FPoints &guard_points,
    geom::FPolygons &guarded_convex_polygons
) {
    int n_polygon = static_cast<int>(convex_polygon.size());
    // Calculate a guard candidate (point WS).
    auto ws = GuardWS(convex_polygon);
    // Calculate a polygon vertex (MDV) with maximum distance from WS.
    int max_dist_id = -1;
    double max_dist = -1.0;
    for (int i = 0; i < n_polygon; ++i) {
        double dist = ws.DistanceTo(convex_polygon[i]);
        if (dist > max_dist) {
            max_dist = dist;
            max_dist_id = i;
        }
    }
    if (vis_model.robustness_radius().has_value()) {
        max_dist += vis_model.robustness_radius().value();
    }
    const auto &mdv = convex_polygon[max_dist_id];
    bool add_guard = false;
    if (!vis_model.vis_radius().has_value() || max_dist < vis_model.vis_radius().value()) {
        if (vis_model.robustness_radius().has_value()) {
            auto vis_reg = vis_model.VisibilityRegion(vis, ws);
            if (vis_reg.has_value()) {
                auto vis_poly = vis_reg->ToPolygon();
                auto result = Clipper2Lib::Difference(
                    {utils::ToClipper(convex_polygon, vis.limits())},
                    {utils::ToClipper(vis_poly, vis.limits())},
                    Clipper2Lib::FillRule::NonZero);
                if (result.empty() || result[0].size() < 3 || geom::Area(utils::FromClipper(result, vis.limits())) < 1e-12) {
                    add_guard = true;
                }
            }
        } else {
            add_guard = true;
        }
    }
    if (add_guard) {
        guard_points.push_back(ws);
        guarded_convex_polygons.push_back(convex_polygon);
        return;
    }
    // Split the polygon into smaller sub-polygons.
    // Identify two points P0 and P1 that define a straight line Q which passes through point WS and is perpendicular to a line determined by points MDV and WS.
    geom::FPoint p0, p1;
    auto u = mdv - ws;
    auto un = (geom::MakePoint(-u.y, u.x) / u.Norm()) * 2.0 * max_dist;
    p0 = ws - un;
    p1 = ws + un;
    // Determine the new polygons.
    geom::Polygon<double> new_polygon_0, new_polygon_1;
    bool hit = false;
    unsigned int hit_cnt = 0;
    auto intersect0_prev = geom::MakePoint(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    // Iterate over polygon's edges and split the polygon into two smaller sub-polygons.
    for (int i = 0; i < n_polygon; ++i) {
        // Points P3 and P4 determine one polygon's edge.
        const auto &pi = convex_polygon[i];
        const auto &pi_next = convex_polygon[(i + 1) % n_polygon];
        // Find intersections of Q and polygon's edges.
        geom::FPoint intersect0, intersect1;
        char int_code = geom::RaySegmentIntersection(p0, p1, pi, pi_next, intersect0, intersect1);
        if ((int_code == 'V' || int_code == 's' || int_code == 'v' || int_code == 'i' || int_code == '1') && intersect0 != intersect0_prev) {
            hit = true;
        }
        intersect0_prev = intersect0;
        // The following assigns polygon's vertices and/or found intersection into one of the two new sub-polygons (in case of the intersection into both of them).
        if (hit_cnt == 0) {
            new_polygon_0.push_back(pi);
            if (hit) {
                new_polygon_0.push_back(intersect0);
                new_polygon_1.push_back(intersect0);
                hit = false;
                ++hit_cnt;
            }
        } else if (hit_cnt == 1) {
            new_polygon_1.push_back(pi);
            if (hit) {
                new_polygon_1.push_back(intersect0);
                new_polygon_0.push_back(intersect0);
                hit = false;
                ++hit_cnt;
            }
        } else if (hit_cnt == 2) {
            new_polygon_0.push_back(pi);
        }
    }
    // Recursively apply this method to the two new sub-polygons.
    AddGuardOrSplit(depth + 1, new_polygon_0, vis, vis_model, guard_points, guarded_convex_polygons);
    AddGuardOrSplit(depth + 1, new_polygon_1, vis, vis_model, guard_points, guarded_convex_polygons);
}

Guards guards::GuardsKA(
    const geom::FPolygons &convex_polygons,
    const Trivis &vis,
    const visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max,
    std::optional<unsigned> random_seed,
    geom::FPolygons *guarded_convex_polygons_out
) {
    Guards guards;
    if (n_max && n_max == 0) return guards;
    geom::FPoints guard_points;
    geom::FPolygons guarded_convex_polygons;
    for (const auto &convex_polygon: convex_polygons) {
        if (vis_model.vis_radius().has_value() || vis_model.robustness_radius().has_value()) {
            AddGuardOrSplit(1, convex_polygon, vis, vis_model, guard_points, guarded_convex_polygons);
        } else {
            guard_points.push_back(GuardWS(convex_polygon));
            guarded_convex_polygons.push_back(convex_polygon);
        }
    }

    auto indices = utils::Range(guard_points.size());
    if (random_seed) {
        std::mt19937 rng(*random_seed);
        std::shuffle(indices.begin(), indices.end(), rng);
    }
    int cnt = 0;
    for (unsigned i: indices) {
        guards.push_back({.point = std::move(guard_points[i])});
        if (guarded_convex_polygons_out) {
            guarded_convex_polygons_out->push_back(std::move(guarded_convex_polygons[i]));
        }
        if (n_max && ++cnt >= n_max) {
            break;
        }
    }
    return guards;
}