/**
 * File:   sample_coverage.cc
 *
 * Date:   12.06.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/coverage/sample_coverage.h"

#include "visis/log/log.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::coverage;

using Clock = trivis::utils::SimpleClock;

void PolytreeToPolygonsWithHoles(
    const Clipper2Lib::PolyTree64 &polytree,
    bool root,
    std::vector<Clipper2Lib::Paths64> &polygons
) {
    int current = static_cast<int>(polygons.size());
    if (!root) {
        auto polygon = polytree.Polygon();
        polygons.push_back({std::move(polygon)});
    }
    for (int i = 0; i < polytree.Count(); ++i) {
        const auto &child = *polytree.Child(i);
        if (child.IsHole()) {
            polygons[current].push_back(child.Polygon());
            for (int j = 0; j < child.Count(); ++j) {
                const auto &grandchild = *child.Child(j);
                PolytreeToPolygonsWithHoles(grandchild, false, polygons);
            }
        } else {
            PolytreeToPolygonsWithHoles(child, false, polygons);
        }
    }
}

std::vector<Clipper2Lib::Paths64> PolytreeToPolygonsWithHoles(
    const Clipper2Lib::PolyTree64 &polytree
) {
    std::vector<Clipper2Lib::Paths64> polygons;
    PolytreeToPolygonsWithHoles(polytree, true, polygons);
    return polygons;
}

std::optional<Clipper2Lib::Paths64> SplitSelfIntersectingPolygon(
    const Clipper2Lib::Path64 &polygon
) {
    int n = static_cast<int>(polygon.size());
    if (n < 4) {
        return std::nullopt;
    }
    bool do_split = false;
    int i, i_next, j, j_next;
    for (i = 0; i < polygon.size(); ++i) {
        i_next = (i + 1 == n) ? 0 : i + 1;
        for (j = i + 2; j < polygon.size(); ++j) {
            j_next = (j + 1 == n) ? 0 : j + 1;
            if (j_next == i) {
                continue;
            }
            if (Clipper2Lib::SegmentsIntersect(polygon[i], polygon[i_next], polygon[j], polygon[j_next], true)) {
                do_split = true;
                break;
            }
        }
        if (do_split) {
            break;
        }
    }
    if (!do_split) {
        return std::nullopt;
    }
    Clipper2Lib::Point64 intersection;
    if (!Clipper2Lib::GetIntersectPoint(polygon[i], polygon[i_next], polygon[j], polygon[j_next], intersection)) {
        return std::nullopt;
    }
    Clipper2Lib::Paths64 new_polygons;
    {   // Scope of the first polygon: (0, ..., i, intersection, j, ..., n - 1).
        Clipper2Lib::Path64 new_poly;
        for (int k = 0; k <= i; ++k) {
            new_poly.push_back(polygon[k]);
        }
        new_poly.push_back(intersection);
        if (j_next > 0) {
            for (int k = j_next; k < n; ++k) {
                new_poly.push_back(polygon[k]);
            }
        }
        Clipper2Lib::StripDuplicates(new_poly, true);
        auto new_poly_split = SplitSelfIntersectingPolygon(new_poly); // Apply recursively to the new polygon.
        if (new_poly_split.has_value()) {
            new_polygons.insert(new_polygons.end(), new_poly_split.value().begin(), new_poly_split.value().end());
        } else {
            new_polygons.push_back(std::move(new_poly));
        }
    }
    {   // Scope of the second polygon: (i + 1, ..., j, intersection).
        Clipper2Lib::Path64 new_poly;
        for (int k = i_next; k <= j; ++k) {
            new_poly.push_back(polygon[k]);
        }
        new_poly.push_back(intersection);
        Clipper2Lib::StripDuplicates(new_poly, true);
        auto new_poly_split = SplitSelfIntersectingPolygon(new_poly); // Apply recursively to the new polygon.
        if (new_poly_split.has_value()) {
            new_polygons.insert(new_polygons.end(), new_poly_split.value().begin(), new_poly_split.value().end());
        } else {
            new_polygons.push_back(std::move(new_poly));
        }
    }
    return new_polygons;
}

Clipper2Lib::Paths64 SplitSelfIntersectingPolygons(
    Clipper2Lib::Paths64 polygons
) {
    Clipper2Lib::Paths64 new_polygons;
    new_polygons.reserve(polygons.size());
    for (auto &polygon: polygons) {
        auto polygon_split = SplitSelfIntersectingPolygon(polygon);
        if (polygon_split.has_value()) {
            new_polygons.insert(new_polygons.end(), polygon_split.value().begin(), polygon_split.value().end());
        } else {
            new_polygons.push_back(std::move(polygon));
        }
    }
    return new_polygons;
}

inline trivis::geom::FPolygons Triangulate(
    const trivis::geom::FPolygons &polygon_with_holes
) {
    trivis::geom::FPolygons triangles;
    trivis::mesh::TriangulateMapCDT(polygon_with_holes, triangles);
    return triangles;
}

void EliminateTriangle(
    int id,
    std::vector<double> &triangle_accum_areas
) {
    if (triangle_accum_areas.empty()) return;
    double area = (id == 0) ? triangle_accum_areas.front() : triangle_accum_areas[id] - triangle_accum_areas[id - 1];
    int n = static_cast<int>(triangle_accum_areas.size());
    for (int i = id; i < n; ++i) {
        triangle_accum_areas[i] -= area;
    }
}

class UncoveredRegion {
public:

    struct Region {
        Clipper2Lib::Paths64 region;
        Clipper2Lib::Rect64 bounds;
        double area;
        trivis::geom::FPolygons triangulation;
        std::vector<double> tri_accum_areas;
        explicit Region(Clipper2Lib::Paths64 polygon_with_holes)
            : region(std::move(polygon_with_holes)),
              bounds(Clipper2Lib::GetBounds(region)),
              area(Clipper2Lib::Area(region)),
              triangulation(),
              tri_accum_areas() {
        }
    };

    struct PointInfo {
        trivis::geom::FPoint point;
        int region_id;
        int triangle_id;
    };

    explicit UncoveredRegion(
        const Clipper2Lib::Paths64 &regions,
        trivis::geom::FLimits limits
    ) {
        Clipper2Lib::PolyTree64 sol_tree;
        _clipper.AddSubject(regions);
        _clipper.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, sol_tree);
        _clipper.Clear();
        auto polygons_with_holes = PolytreeToPolygonsWithHoles(sol_tree);
        _regions.reserve(polygons_with_holes.size());
        for (auto &polygon_with_holes: polygons_with_holes) {
            auto reg = SplitSelfIntersectingPolygons(std::move(polygon_with_holes));
            _regions.emplace_back(std::move(reg));
        }
        _limits = limits;
    }

    UncoveredRegion(const UncoveredRegion &other) {
        _regions = other.regions();
        _limits = other.limits();
    }

    UncoveredRegion &operator=(const UncoveredRegion &other) {
        _regions = other.regions();
        _limits = other.limits();
        return *this;
    }

    ~UncoveredRegion() = default;

    void ClipOff(
        const Clipper2Lib::Paths64 &subtrahend,
        std::optional<double> simplify_paths_eps = std::nullopt
    ) {
        std::vector<int> to_remove;
        auto bounds = Clipper2Lib::GetBounds(subtrahend);
        for (int i = 0; i < _regions.size(); ++i) {
            auto &region = _regions[i];
            if (!bounds.Intersects(region.bounds)) {
                continue; // Skip regions that do not intersect the subtrahend.
            }
            // Compute the difference of the region and the subtrahend.
            Clipper2Lib::Paths64 sol;
            _clipper.AddSubject(region.region);
            _clipper.AddClip(subtrahend);
            _clipper.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, sol);
            _clipper.Clear();
            if (sol.empty()) {
                to_remove.push_back(i);
                continue; // Nothing left of the region.
            }
            // Simplify the result to improve robustness (otherwise Triangle may loop or crash).
            if (simplify_paths_eps.has_value()) {
                sol = Clipper2Lib::SimplifyPaths(sol, simplify_paths_eps.value());
            }
            // Split self-intersecting polygons to improve robustness (otherwise Triangle may loop or crash).
            sol = SplitSelfIntersectingPolygons(sol);
            if (sol.empty()) {
                to_remove.push_back(i);
                continue; // Nothing left of the region.
            }
            // Compute the union after simplification and get the result as a sequence of polygons with holes.
            Clipper2Lib::PolyTree64 sol_tree;
            _clipper.AddSubject(sol);
            _clipper.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, sol_tree);
            _clipper.Clear();
            auto polygons_with_holes = PolytreeToPolygonsWithHoles(sol_tree);
            if (polygons_with_holes.empty()) {
                to_remove.push_back(i);
                continue; // Nothing left of the region.
            }
            for (int j = 0; j < polygons_with_holes.size(); ++j) {
                if (j == 0) {
                    // Replace the current region with the first polygon with holes.
                    region = Region(std::move(polygons_with_holes[j]));
                } else {
                    // Append the other polygons with holes.
                    _regions.emplace_back(std::move(polygons_with_holes[j]));
                }
            }
        }
        // Remove regions that were reduced to nothing.
        for (int i = static_cast<int>(to_remove.size()) - 1; i >= 0; --i) {
            _regions.erase(_regions.begin() + to_remove[i]);
        }
    }

    [[nodiscard]] double Area() const {
        double area = 0.0;
        for (const auto &region: _regions) {
            area += region.area;
        }
        return area;
    }

    [[nodiscard]] PointInfo RandomSample(
        std::mt19937 &rng
    ) {
        int reg_id = 0;
        if (_regions.size() > 1) {
            std::vector<double> reg_accum_areas;
            reg_accum_areas.reserve(_regions.size());
            for (const auto &region: _regions) {
                reg_accum_areas.push_back(region.area + (reg_accum_areas.empty() ? 0.0 : reg_accum_areas.back()));
            }
            auto r = std::uniform_real_distribution<double>(0.0, reg_accum_areas.back())(rng);
            for (; reg_id < _regions.size(); ++reg_id) {
                if (reg_accum_areas[reg_id] >= r) {
                    break;
                }
            }
        }
        int triangle_id;
        trivis::geom::FPoint point;
        auto &region = _regions[reg_id];
        if (region.triangulation.empty()) {
            region.triangulation = Triangulate(trivis::utils::FromClipper(region.region, _limits));
            point = trivis::utils::UniformRandomPointInRandomTriangle(region.triangulation, region.tri_accum_areas, rng, &triangle_id);
        } else {
            const auto &tri_accum_areas = region.tri_accum_areas;
            point = trivis::utils::UniformRandomPointInRandomTriangle(region.triangulation, tri_accum_areas, rng, &triangle_id);
        }
        return PointInfo{.point=point, .region_id=reg_id, .triangle_id=triangle_id};
    }

    void EliminateTriangle(
        int region_id,
        int triangle_id
    ) {
        ::EliminateTriangle(triangle_id, _regions[region_id].tri_accum_areas);
    }

    [[nodiscard]] Clipper2Lib::Paths64 ToUncoveredRegion() const {
        Clipper2Lib::Paths64 uncovered_region;
        for (const auto &region: _regions) {
            uncovered_region.insert(uncovered_region.end(), region.region.begin(), region.region.end());
        }
        return uncovered_region;
    }

    [[nodiscard]] const trivis::geom::FLimits &limits() const {
        return _limits;
    }

    [[nodiscard]] const std::vector<Region> &regions() const {
        return _regions;
    }

private:
    trivis::geom::FLimits _limits;
    Clipper2Lib::Clipper64 _clipper;
    std::vector<Region> _regions;
};

inline Clipper2Lib::Rect64 Rectangle(
    const coverage::ClipperLimits &lim
) {
    return {lim.x_min, lim.y_min, lim.x_max, lim.y_max};
}

inline Clipper2Lib::Rect64 Rectangle(
    const trivis::geom::FPoint &seed,
    double radius,
    const trivis::geom::FLimits &lim
) {
    long r = trivis::utils::ToClipper(radius, lim);
    auto p = trivis::utils::ToClipper({seed}, lim).front();
    return Rectangle(coverage::ClipperLimits{p.x - r, p.y - r, p.x + r, p.y + r});
}

inline Clipper2Lib::Paths64 GetReducedUncoveredRegion(
    const Clipper2Lib::Paths64 &uncovered_region,
    const trivis::geom::FPoint &seed,
    double radius,
    const trivis::geom::FLimits &lim
) {
    if (radius <= 0.0 || radius >= std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min)) {
        return uncovered_region;
    }
    return Clipper2Lib::RectClip(Rectangle(seed, radius, lim), uncovered_region);
}

void coverage::SampleCoverageRandom(
    Coverage &coverage,
    guards::Guards &guards,
    Clipper2Lib::Paths64 &uncovered_region,
    const trivis::Trivis &vis,
    double target_regions_c_area,
    visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max,
    double min_coverage_ratio,
    unsigned random_seed
) {
    if (n_max && n_max == 0) return;
    // Prepare structures for random point generation.
    std::mt19937 rng(random_seed);
    UncoveredRegion ur(uncovered_region, vis.limits());
    UncoveredRegion ur_const = ur;
    // Prepare goal ratio.
    double uncovered_ratio = ur.Area() / target_regions_c_area;
    double uncovered_ratio_goal = 1.0 - min_coverage_ratio;
    // Main loop:
    int cnt = 0;
    while (uncovered_ratio > uncovered_ratio_goal) {
        // Get the sample point and region.
        auto sample_point_info = ur_const.RandomSample(rng);
        auto &sample_point = sample_point_info.point;
        auto sample_point_pl = vis.LocatePoint(sample_point);
        if (!sample_point_pl.has_value()) {
            LOGF_WRN("Visibility region could not be computed from sample point " << sample_point << ".");
            continue;
        }
        auto sample_region = vis_model.VisibilityRegion(vis, sample_point, sample_point_pl.value());
        if (!sample_region.has_value()) {
            LOGF_WRN("Visibility region could not be computed from sample point " << sample_point << " despite the sample point was located.");
            continue;
        }
        // Compute approximation of the sample region.
        auto sample_region_with_appx = MakeVisRegionWithApprox(vis.limits(), std::move(sample_region.value()), vis_model.sample_arc_edges_angle());
        // Update uncovered region and ratio.
        ur.ClipOff({sample_region_with_appx.approx.clipper}, 1e-3);
        uncovered_ratio = ur.Area() / target_regions_c_area;
        // Move the sample point and region to the guards and coverage, respectively.
        coverage.push_back(std::move(sample_region_with_appx));
        guards.push_back(guards::Guard{std::move(sample_point), false, false, std::nullopt, std::nullopt, sample_point_pl->tri_id});
        // Check the n_max stopping condition.
        if (n_max && ++cnt >= n_max) {
            break;
        }
    }
    uncovered_region = ur.ToUncoveredRegion();
}

void coverage::SampleCoverageInformed(
    Coverage &coverage,
    guards::Guards &guards,
    Clipper2Lib::Paths64 &uncovered_region,
    const trivis::Trivis &vis,
    double target_regions_c_area,
    visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max,
    double min_coverage_ratio,
    unsigned random_seed
) {
    if (n_max && n_max == 0) return;
    // Prepare structures for random point generation (Part 1).
    std::mt19937 rng(random_seed);
    UncoveredRegion ur(uncovered_region, vis.limits());
    // Prepare goal ratio.
    double uncovered_ratio = ur.Area() / target_regions_c_area;
    double uncovered_ratio_goal = 1.0 - min_coverage_ratio;
    // Main loop:
    int cnt = 0;
    while (uncovered_ratio > uncovered_ratio_goal) {
        // Get the sample point and region.
        auto sample_point_info = ur.RandomSample(rng);
        auto &sample_point = sample_point_info.point;
        auto sample_point_pl = vis.LocatePoint(sample_point);
        if (!sample_point_pl.has_value()) {
            LOGF_WRN("Visibility region could not be computed from sample point " << sample_point << ".");
            ur.EliminateTriangle(sample_point_info.region_id, sample_point_info.triangle_id); // This triangle will not be selected again.
            continue;
        }
        auto sample_region = vis_model.VisibilityRegion(vis, sample_point, sample_point_pl.value());
        if (!sample_region.has_value()) {
            LOGF_WRN("Visibility region could not be computed from sample point " << sample_point << " despite the sample point was located.");
            continue;
        }
        // Compute approximation of the sample region.
        auto sample_region_with_appx = MakeVisRegionWithApprox(vis.limits(), std::move(sample_region.value()), vis_model.sample_arc_edges_angle());
        // Update uncovered region and ratio.
        ur.ClipOff({sample_region_with_appx.approx.clipper}, 1e-3);
        uncovered_ratio = ur.Area() / target_regions_c_area;
        // Move the sample point and region to the guards and coverage, respectively.
        coverage.push_back(std::move(sample_region_with_appx));
        guards.push_back(guards::Guard{std::move(sample_point), false, false, std::nullopt, std::nullopt, sample_point_pl->tri_id});
        // Check the n_max stopping condition.
        if (n_max && ++cnt >= n_max) {
            break;
        }
    }
    uncovered_region = ur.ToUncoveredRegion();
}

void coverage::SampleCoverageInformedDual(
    Coverage &coverage,
    guards::Guards &guards,
    Clipper2Lib::Paths64 &uncovered_region,
    const trivis::Trivis &vis,
    double target_regions_c_area,
    visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max,
    double min_coverage_ratio,
    unsigned random_seed,
    std::optional<unsigned> n_dual_samples,
    std::optional<double> n_dual_samples_per_square_unit
) {
    if (!n_dual_samples.has_value() && !n_dual_samples_per_square_unit.has_value()) {
        LOGF_WRN("Neither n_dual_samples nor n_dual_samples_per_square_unit is set. Using n_dual_samples = 10.");
        n_dual_samples = 10;
    }
    if (n_max && n_max == 0) return;
    // Prepare structures for random point generation (Part 1).
    std::mt19937 rng(random_seed);
    UncoveredRegion ur(uncovered_region, vis.limits());
    // Prepare goal ratio.
    double uncovered_ratio = ur.Area() / target_regions_c_area;
    double uncovered_ratio_goal = 1.0 - min_coverage_ratio;
    // Main loop:
    int cnt = 0;
    while (uncovered_ratio > uncovered_ratio_goal) {
        // Get the sample point and region.
        auto sample_point_info = ur.RandomSample(rng);
        auto &sample_point = sample_point_info.point;
        auto sample_point_pl = vis.LocatePoint(sample_point);
        if (!sample_point_pl.has_value()) {
            LOGF_WRN("Visibility region could not be computed from sample point " << sample_point << ".");
            ur.EliminateTriangle(sample_point_info.region_id, sample_point_info.triangle_id); // This triangle will not be selected again.
            continue;
        }
        auto sample_region = vis_model.VisibilityRegion(vis, sample_point, sample_point_pl.value());
        if (!sample_region.has_value()) {
            LOGF_WRN("Visibility region could not be computed from sample point " << sample_point << " despite the sample point was located.");
            continue;
        }
        // Compute approximation of the sample region.
        auto sample_region_with_appx = MakeVisRegionWithApprox(vis.limits(), std::move(sample_region.value()), vis_model.sample_arc_edges_angle());
        double reduction_radius = vis_model.vis_radius().has_value() ? 2.0 * vis_model.vis_radius().value() : -1.0;
        // Get reduced uncovered region.
        auto uncovered_region_reduced = GetReducedUncoveredRegion(ur.ToUncoveredRegion(), sample_point, reduction_radius, vis.limits());
        // Compute the area of intersection with the uncovered region.
        auto best_area = Clipper2Lib::Area(Clipper2Lib::Intersect(uncovered_region_reduced, {sample_region_with_appx.approx.clipper}, Clipper2Lib::FillRule::NonZero));
        // Prepare structures for random point generation (dual).
        std::vector<double> sample_region_triangle_accum_areas;
        auto sample_region_triangulation = Triangulate({sample_region_with_appx.approx.polygon});
        unsigned n_dual_samples_final = n_dual_samples.value_or(0);
        if (n_dual_samples_per_square_unit.has_value()) {
            double area = trivis::geom::Area(sample_region_with_appx.approx.polygon);
            n_dual_samples_final = std::max(n_dual_samples_final, static_cast<unsigned>(std::ceil(area * n_dual_samples_per_square_unit.value())));
        }
        int n_err = 0;
        // Dual loop:
        for (int k = 0; k < n_dual_samples_final;) {
            // Get the dual sample point and region.
            int k_sample_triangle_id;
            auto k_sample_point = trivis::utils::UniformRandomPointInRandomTriangle(sample_region_triangulation, sample_region_triangle_accum_areas, rng, &k_sample_triangle_id);
            auto k_sample_point_pl = vis.LocatePoint(k_sample_point);
            if (!k_sample_point_pl.has_value()) {
                LOGF_WRN("Visibility region could not be computed from dual sample point " << k_sample_point << ".");
                ++n_err;
                if (n_err > n_dual_samples_final) {
                    LOGF_WRN("Too many errors when computing visibility regions for dual samples. Stopping the dual loop.");
                    break;
                }
                EliminateTriangle(k_sample_triangle_id, sample_region_triangle_accum_areas); // This triangle will not be selected again.
                continue;
            }
            n_err = 0;
            auto k_sample_region = vis_model.VisibilityRegion(vis, k_sample_point, k_sample_point_pl.value());
            if (!k_sample_region.has_value()) {
                LOGF_WRN("Visibility region could not be computed from dual sample point " << k_sample_point << " despite the dual sample point was located.");
                continue;
            }
            // Compute approximation of the sample region.
            auto k_sample_region_with_appx = MakeVisRegionWithApprox(vis.limits(), std::move(k_sample_region.value()), vis_model.sample_arc_edges_angle());
            // Compute the area of intersection with the uncovered region.
            auto k_area = Clipper2Lib::Area(Clipper2Lib::Intersect(uncovered_region_reduced, {k_sample_region_with_appx.approx.clipper}, Clipper2Lib::FillRule::NonZero));
            // Evaluate the dual case.
            if (k_area > best_area) {
                best_area = k_area;
                sample_point = std::move(k_sample_point);
                sample_point_pl = std::move(k_sample_point_pl);
                sample_region_with_appx = std::move(k_sample_region_with_appx);
            }
            ++k;
        }
        // Update uncovered region and ratio.
        ur.ClipOff({sample_region_with_appx.approx.clipper}, 1e-3);
        uncovered_ratio = ur.Area() / target_regions_c_area;
        // Move the sample point and region to the guards and coverage, respectively.
        coverage.push_back(std::move(sample_region_with_appx));
        guards.push_back(guards::Guard{std::move(sample_point), false, false, std::nullopt, std::nullopt, sample_point_pl->tri_id});
        // Check the n_max stopping condition.
        if (n_max && ++cnt >= n_max) {
            break;
        }
    }
    uncovered_region = ur.ToUncoveredRegion();
}
