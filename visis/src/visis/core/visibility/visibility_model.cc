/**
 * File:   visibility_model.cc
 *
 * Date:   21.05.2024
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/visibility/visibility_model.h"

#include "visis/log/log.h"

#include "trivis/geom/intersections.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::visibility;

inline bool IsSetAndPositive(const std::optional<double> &opt_value) {
    return opt_value.has_value() && std::isfinite(opt_value.value()) && opt_value.value() > 0.0;
}

template<typename T>
inline std::string ToString(const std::optional<T> &opt_value) {
    return opt_value.has_value() ? std::to_string(opt_value.value()) : "nullopt";
}

VisibilityModel::VisibilityModel(
    std::optional<double> vis_radius,
    std::optional<double> robustness_radius,
    std::optional<double> robustness_sample_dist,
    std::optional<double> sample_arc_edges_angle,
    std::optional<double> sample_arc_edges_dist
) : _vis_radius(vis_radius),
    _robustness_radius(robustness_radius),
    _robustness_sample_dist(robustness_sample_dist) {
    if (sample_arc_edges_angle.has_value()) {
        _sample_arc_edges_angle = sample_arc_edges_angle;
    } else if (sample_arc_edges_dist.has_value() && vis_radius.has_value()) {
        _sample_arc_edges_angle = sample_arc_edges_dist.value() / vis_radius.value();
    }
}

bool VisibilityModel::CheckValidity() const {
    if (_vis_radius.has_value()) {
        if (!IsSetAndPositive(_vis_radius)) {
            LOGF_ERR("Visibility radius must be positive! The value is " << ToString(_vis_radius) << ".");
            return false;
        }
    }
    if (_robustness_radius.has_value()) {
        if (!IsSetAndPositive(_robustness_radius)) {
            LOGF_ERR("Robustness radius must be positive! The value is " << ToString(_robustness_radius) << ".");
            return false;
        }
        if (!IsSetAndPositive(_robustness_sample_dist)) {
            LOGF_ERR("Robustness sample distance must be positive! The value is " << ToString(_robustness_sample_dist) << ".");
            return false;
        }
        if (_vis_radius.has_value()) {
            if (!IsSetAndPositive(_sample_arc_edges_angle)) {
                LOGF_ERR("Sample arc edges angle must be positive! The value is " << ToString(_sample_arc_edges_angle) << ".");
                return false;
            }
        }
    }
    return true;
}

std::optional<trivis::RadialVisibilityRegion> RobustVisibilityRegion(
    const trivis::Trivis &vis,
    trivis::RadialVisibilityRegion p_reg,
    const std::optional<double> &vis_radius,
    double robustness_radius,
    double robustness_sample_dist,
    const std::optional<double> &sample_arc_edges_angle,
    trivis::Trivis::ExpansionStats *stats = nullptr
) {
    // Compute the shortest distance from the seed to the boundary of the visibility region.
    double min_dist_sq = (vis_radius.has_value()) ? vis_radius.value() * vis_radius.value() : std::numeric_limits<double>::infinity();
    for (int i_prev = static_cast<int>(p_reg.vertices.size()) - 1, i = 0; i < p_reg.vertices.size(); i_prev = i++) {
        const auto &vi_prev = p_reg.vertices[i_prev];
        const auto &vi = p_reg.vertices[i];
        if (vi.edge_flag < 0) {
            continue; // free edge
        }
        double dist_sq = trivis::geom::PointSegmentSquaredDistance(p_reg.seed, vi_prev.point, vi.point);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
        }
    }
    // Adapt the robustness radius to the visibility region boundary.
    if (std::isfinite(min_dist_sq)) {
        robustness_radius = std::min(robustness_radius, std::sqrt(min_dist_sq) - 1e-6);
    }
    // Sample the arcs of the visibility region.
    if (vis_radius.has_value()) {
        assert(sample_arc_edges_angle.has_value());
        p_reg.SampleArcEdges(sample_arc_edges_angle.value());
    }
    Clipper2Lib::Paths64 clipper_polygons;
    {   // Construct the visibility regions from the robustness circle.
        trivis::RadialVisibilityRegion robustness_circle{.radius = robustness_radius, .seed = p_reg.seed};
        robustness_circle.SampleArcEdges(robustness_sample_dist / robustness_radius);
        clipper_polygons.reserve(robustness_circle.vertices.size());
        for (const auto &v: robustness_circle.vertices) {
            auto v_pl = vis.LocatePoint(v.point);
            if (!v_pl.has_value()) {
                continue;
            }
            if (!vis.IsVisible(v.point, v_pl.value(), p_reg.seed)) {
                continue;
            }
            auto v_abs_reg = vis.VisibilityRegion(v.point, v_pl.value(), vis_radius, stats);
            auto v_reg = vis.ToRadialVisibilityRegion(v_abs_reg);
            if (vis_radius.has_value()) {
                v_reg.IntersectWithCircleCenteredAtSeed(vis_radius);
                v_reg.SampleArcEdges(sample_arc_edges_angle.value());
            }
            clipper_polygons.push_back(trivis::utils::ToClipper(v_reg.ToPolygon(), vis.limits()));
        }
    }
    Clipper2Lib::Paths64 clipper_solution = {trivis::utils::ToClipper(p_reg.ToPolygon(), vis.limits())};
    {   // Compute the intersection of all the visibility regions.
        Clipper2Lib::Clipper64 clipper;
        for (const auto &clipper_poly: clipper_polygons) {
            clipper.AddSubject(clipper_solution);
            clipper.AddClip({clipper_poly});
            clipper.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, clipper_solution);
            clipper.Clear();
            if (clipper_solution.empty() || clipper_solution[0].size() < 3) {
                return std::nullopt;
            }
        }
    }
    // Convert the intersection result to RadialVisibilityRegion.
    auto solution_poly = trivis::utils::FromClipper(clipper_solution[0], vis.limits());
    trivis::RadialVisibilityRegion ret{.radius = vis_radius, .seed_id = p_reg.seed_id, .seed = p_reg.seed};
    ret.vertices.reserve(solution_poly.size());
    for (const auto &sp: solution_poly) {
        ret.vertices.push_back(trivis::VisibilityRegionVertex{.vertex_flag = -2, .edge_flag = -4, .point = sp});
    }
    return ret;
}

std::optional<trivis::RadialVisibilityRegion> VisibilityModel::VisibilityRegion(
    const trivis::Trivis &vis,
    const trivis::geom::FPoint &p,
    const trivis::Trivis::PointLocationResult &pl,
    trivis::Trivis::ExpansionStats *stats
) const {
    auto abs_reg = vis.VisibilityRegion(p, pl, _vis_radius, stats);
    auto reg = vis.ToRadialVisibilityRegion(abs_reg);
    if (_vis_radius.has_value()) {
        reg.IntersectWithCircleCenteredAtSeed(_vis_radius);
    }
    if (_robustness_radius.has_value()) {
        assert(_robustness_sample_dist.has_value() && _sample_arc_edges_angle.has_value());
        return RobustVisibilityRegion(vis, std::move(reg), _vis_radius, _robustness_radius.value(), _robustness_sample_dist.value(), _sample_arc_edges_angle, stats);
    } else {
        return reg;
    }
}

std::optional<trivis::RadialVisibilityRegion> VisibilityModel::VisibilityRegion(
    const trivis::Trivis &vis,
    const trivis::geom::FPoint &p,
    trivis::Trivis::ExpansionStats *stats
) const {
    auto pl = vis.LocatePoint(p);
    if (!pl.has_value()) {
        return std::nullopt;
    }
    return VisibilityRegion(vis, p, pl.value());
}

std::optional<trivis::RadialVisibilityRegion> VisibilityModel::VisibilityRegion(
    const trivis::Trivis &vis,
    const trivis::geom::FPoint &p,
    int p_tri_id,
    trivis::Trivis::ExpansionStats *stats
) const {
    auto abs_reg = vis.VisibilityRegion(p, p_tri_id, _vis_radius, stats);
    auto reg = vis.ToRadialVisibilityRegion(abs_reg);
    if (_vis_radius.has_value()) {
        reg.IntersectWithCircleCenteredAtSeed(_vis_radius);
    }
    if (_robustness_radius.has_value()) {
        assert(_robustness_sample_dist.has_value());
        return RobustVisibilityRegion(vis, std::move(reg), _vis_radius, _robustness_radius.value(), _robustness_sample_dist.value(), _sample_arc_edges_angle, stats);
    } else {
        return reg;
    }
}

std::optional<trivis::RadialVisibilityRegion> VisibilityModel::VisibilityRegion(
    const trivis::Trivis &vis,
    int ver_id,
    trivis::Trivis::ExpansionStats *stats
) const {
    auto abs_reg = vis.VisibilityRegion(ver_id, _vis_radius, stats);
    auto reg = vis.ToRadialVisibilityRegion(abs_reg);
    if (_vis_radius.has_value()) {
        reg.IntersectWithCircleCenteredAtSeed(_vis_radius);
    }
    if (_robustness_radius.has_value()) {
        assert(_robustness_sample_dist.has_value() && _sample_arc_edges_angle.has_value());
        return RobustVisibilityRegion(vis, std::move(reg), _vis_radius, _robustness_radius.value(), _robustness_sample_dist.value(), _sample_arc_edges_angle, stats);
    } else {
        return reg;
    }
}

