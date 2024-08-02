/**
 * File:   visis.cc
 *
 * Date:   11.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/visis.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "visis/core/coverage/sample_coverage.h"
#include "visis/core/coverage/filter_coverage.h"

#include "visis/core/guards/guards_all.h"
#include "visis/core/guards/process_guards.h"

#include "visis/drawing/fancy_drawing.h"
#include "visis/drawing/random_colors.h"

#include "visis/data/find_file.h"
#include "visis/data/load_map.h"

#include "visis/statistics/evaluate_vis.h"
#include "visis/statistics/statistics.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;

using namespace trivis;

using Clock = trivis::utils::SimpleClock;
using PTree = boost::property_tree::ptree;

namespace fs = boost::filesystem;
namespace dr = drawing;

bool Visis::SetVisibilityModel(
    std::optional<double> vis_radius,
    std::optional<double> robustness_radius,
    std::optional<double> robustness_sample_dist,
    std::optional<double> sample_arc_edges_angle,
    std::optional<double> sample_arc_edges_dist
) {
    if (vis_radius.has_value() && _rob_radius.has_value() && vis_radius.value() <= _rob_radius.value()) {
        LOGF_ERR("Visibility radius must be bigger than robot radius!");
        return false;
    }
    _vis_model = visibility::VisibilityModel(vis_radius, robustness_radius, robustness_sample_dist, sample_arc_edges_angle, sample_arc_edges_dist);
    if (!_vis_model.CheckValidity()) {
        LOGF_ERR("Visibility model is not valid!");
        return false;
    }
    _records.Put("vis_radius", vis_radius ? vis_radius.value() : std::numeric_limits<double>::infinity());
    _records.Put("robustness_radius", robustness_radius ? robustness_radius.value() : 0.0);
    _records.Put("robustness_sample_dist", robustness_sample_dist ? robustness_sample_dist.value() : std::numeric_limits<double>::quiet_NaN());
    _records.Put("sample_arc_edges_angle", sample_arc_edges_angle ? sample_arc_edges_angle.value() : std::numeric_limits<double>::quiet_NaN());
    _records.Put("sample_arc_edges_dist", sample_arc_edges_dist ? sample_arc_edges_dist.value() : std::numeric_limits<double>::quiet_NaN());
    return true;
}

bool Visis::SetRobotRadius(double radius) {
    if (std::isnan(radius) || std::isinf(radius) || radius < 0.0) {
        LOGF_ERR("Robot radius must be positive finite (or zero)!");
        return false;
    }
    if (_vis_model.vis_radius().has_value() && radius >= _vis_model.vis_radius().value()) {
        LOGF_ERR("Robot radius must be smaller than visibility radius!");
        return false;
    }
    _records.Put("rob_radius", radius);
    if (radius == 0.0) {
        _rob_radius = std::nullopt;
        return true;
    }
    _rob_radius = radius;
    return true;
}

bool Visis::LoadMap(
    const std::string &map_path_or_name,
    const std::optional<std::string> &map_ext,
    const std::optional<std::string> &map_dir,
    const std::optional<double> &map_scale
) {
    LOGF_INF("======== Loading map.");
    Clock clock;

    // Find map file.
    auto map_path = data::FindFile(map_path_or_name, map_ext, map_dir);
    if (map_path.empty()) {
        return false;
    }

    // Load the map and set it to Vis.
    { // map scope
        LOGF_INF("Loading the map.");
        auto map = data::LoadPolyMapSafely(map_path.string(), map_scale);
        if (!map) {
            return false;
        }
        LOGF_INF("Preprocessing the map.");
        // === Bellow is some optional map preprocessing. ===
        // Warning: order of the following matters!
        map->ShiftToOrigin(); // Subtracts min X and Y coordinates from all points.
        map->RemoveDuplicatePoints(); // Removes all consecutive identical points.
        map->SimplifyWeaklySelfIntersectingPolygons(); // Splits all weakly simple polygons to multiple (touching) strongly simple polygons.
        map->RemoveCollinearPoints(); // Removes all consecutive collinear points.
        // ==================================================
        _regions.SetMap(std::move(*map)); // Set the map.
        // cannot use map anymore ! (it was moved)
    }
    _vis.SetMap(_regions.map()); // Set the map to Vis instance.
    double time = clock.TimeInSeconds();
    data::PTreeGeom rec;
    rec.Put("name", map_path.replace_extension("").filename().string());
    rec.Put("path", map_path);
    rec.Put("scale", _regions.map().scale());
    rec.Put("load_time", time);
    _records.PutChild("map", rec);
    LOGF_INF("==DONE== Loading map. | Time " << time << " s.");
    return true;
}

bool Visis::InitVis(
    double bucket_size,
    bool optimize_buckets
) {
    if (!_regions.has_map()) {
        LOGF_ERR("Cannot initialize Vis. Visis does not have a map. Call LoadMap.");
        return false;
    }
    LOGF_INF("======== Initializing Vis.");
    Clock clock;
    _vis.ConstructMeshCDT();
    double time_mesh = clock.TimeInSeconds();
    clock.Restart();
    _vis.FillPointLocationBuckets(bucket_size);
    double time_fill_bucketing = clock.TimeInSeconds();
    clock.Restart();
    if (optimize_buckets) {
        _vis.OptimizePointLocationBucketTriangles();
    }
    double time_optimize_buckets = clock.TimeInSeconds();
    double time_total = time_mesh + time_fill_bucketing + time_optimize_buckets;
    data::PTreeGeom rec;
    rec.Put("mesh_type", "CDT");
    rec.Put("bucket_size", bucket_size);
    rec.Put("buckets_optimized", optimize_buckets);
    rec.Put("time_mesh", time_mesh);
    rec.Put("time_fill_bucketing ", time_fill_bucketing);
    rec.Put("time_optimize_buckets", time_optimize_buckets);
    rec.Put("time_total", time_total);
    _records.PutChild("vis", rec);
    _has_vis = true;
    LOGF_INF("==DONE== Initializing Vis. | Time " << time_total << " s.");
    return true;
}

bool Visis::ConstructReflexVisibilityGraph() {
    if (!_has_vis) {
        LOGF_ERR("Cannot construct the reflex visibility graph. Visis does not have a Vis instance. Call InitVis.");
        return false;
    }
    LOGF_INF("======== Constructing the reflex visibility graph.");
    Clock clock;
    auto rec_opt = _path_finder.ConstructReflexVisibilityGraph(_vis);
    if (!rec_opt) {
        LOGF_ERR("Path finder could not construct the reflex visibility graph.");
        return false;
    }
    _records.PutChild("vis_graph_reflex", *rec_opt);
    LOGF_INF("==DONE== Constructing the reflex visibility graph. | Time " << clock.TimeInSeconds() << " s.");
    return true;
}

bool Visis::PrecomputeReflexShortestPaths() {
    if (!_path_finder.has_vis_graph_reflex()) {
        LOGF_ERR("Cannot precompute the reflex shortest paths. Visis does not have a visibility graph. Call ConstructReflexVisibilityGraph.");
        return false;
    }
    LOGF_INF("======== Precomputing the reflex shortest paths.");
    Clock clock;
    auto rec_opt = _path_finder.PrecomputeReflexShortestPaths();
    if (!rec_opt) {
        LOGF_ERR("Path finder could not be precompute the reflex shortest paths.");
        return false;
    }
    _records.PutChild("shortest_paths_reflex", *rec_opt);
    LOGF_INF("==DONE== Precomputing the reflex shortest paths. | Time " << clock.TimeInSeconds() << " s.");
    return true;
}

bool Visis::SetTargetRegions(
    double min_coverage_ratio,
    std::optional<std::vector<trivis::geom::FPolygons>> target_regions
) {
    if (!(0.0 < min_coverage_ratio && min_coverage_ratio < 1.0)) {
        LOGF_ERR("Cannot set target regions. Coverage ratio must be positive real number smaller than 1!");
        return false;
    }
    if (!_regions.has_map()) {
        LOGF_ERR("Cannot set target regions. Visis does not have a map. Call LoadMap.");
        return false;
    }
    LOGF_INF("======== Setting target regions.");
    Clock clock;
    _regions.SetTargetRegions(min_coverage_ratio, std::move(target_regions));
    double time = clock.TimeInSeconds();
    double area = 0.0;
    for (const auto &reg: _regions.target_regions()) {
        area += Area(reg);
    }
    data::PTreeGeom rec;
    rec.Put("min_coverage_ratio", _regions.min_coverage_ratio());
    rec.Put("cnt", _regions.target_regions().size());
    rec.Put("area", area);
    rec.Put("map_coverage", _regions.target_regions_c_area() / _regions.map_c_area());
    rec.Put("time", time);
    _records.PutChild("target_regions", rec);
    LOGF_INF("==DONE== Setting target regions. | Time " << time << " s.");
    return true;
}

bool Visis::SetWeightRegions(
    std::optional<std::vector<trivis::geom::FPolygons>> weight_regions,
    std::optional<std::vector<double>> weights
) {
    if (!weight_regions && weights) {
        LOGF_ERR("Cannot set weight regions. Weight regions must be provided if weights are provided!");
        return false;
    }
    if (weight_regions && weights && weight_regions->size() != weights->size()) {
        LOGF_ERR("Cannot set weight regions. Weight regions and weights must have the same size!");
        return false;
    }
    if (!_regions.has_target_regions()) {
        LOGF_ERR("Cannot set weight regions. Visis does not have target regions. Call SetTargetRegions.");
        return false;
    }
    LOGF_INF("======== Setting target regions.");
    Clock clock;
    _regions.SetWeightRegions(weight_regions, weights);
    double time = clock.TimeInSeconds();
    data::PTreeGeom rec;
    rec.Put("cnt", _regions.n_weight_regions());
    rec.Put("time", time);
    _records.PutChild("weight_regions", rec);
    LOGF_INF("==DONE== Setting weight regions. | Time " << time << " s.");
    return true;
}

bool Visis::BucketizeTargetRegions(
    std::optional<double> bucket_ratio,
    std::optional<double> bucket_size
) {
    if (!bucket_ratio && !bucket_size) {
        LOGF_ERR("Cannot bucketize target regions. Either bucket ratio or bucket size must be set.");
        return false;
    }
    if (bucket_ratio && bucket_size) {
        LOGF_ERR("Cannot bucketize target regions. Both bucket ratio or bucket size cannot be set.");
        return false;
    }
    if (bucket_ratio && !(0.0 < bucket_ratio && bucket_ratio <= 1.0)) {
        LOGF_ERR("Cannot bucketize target regions. Bucket ratio must be positive real number smaller than or equal to 1 .");
        return false;
    }
    if (!_regions.has_target_regions()) {
        LOGF_ERR("Cannot bucketize target regions. Visis does not target regions. Call SetTargetRegions.");
        return false;
    }
    LOGF_INF("======== Bucketing target regions.");
    Clock clock;
    _regions.BucketizeTargetRegions(bucket_ratio, bucket_size);
    double time = clock.TimeInSeconds();
    const auto &lim = _regions.map().limits();
    double bucket_size_real = utils::FromClipper(_regions.target_bucket_size_c(), lim);
    double bucket_ratio_real = bucket_size_real / std::max(lim.x_max - lim.x_min, lim.y_max - lim.y_min);
    data::PTreeGeom rec;
    rec.Put("bucket_ratio_in", bucket_ratio ? *bucket_ratio : std::numeric_limits<double>::quiet_NaN());
    rec.Put("bucket_size_in", bucket_size ? *bucket_size : std::numeric_limits<double>::quiet_NaN());
    rec.Put("bucket_ratio_real", bucket_ratio_real);
    rec.Put("bucket_size_real", bucket_size_real);
    rec.Put("bucket_cnt", _regions.target_buckets().size());
    rec.Put("time", time);
    _records.PutChild("target_regions_bucketing", rec);
    LOGF_INF("==DONE== Bucketing target regions. | Time " << time << " s.");
    return true;
}

bool Visis::BucketizeWeightRegions(
    std::optional<double> bucket_ratio,
    std::optional<double> bucket_size
) {
    if (!bucket_ratio && !bucket_size) {
        LOGF_ERR("Cannot bucketize weight regions. Either bucket ratio or bucket size must be set.");
        return false;
    }
    if (bucket_ratio && bucket_size) {
        LOGF_ERR("Cannot bucketize weight regions. Both bucket ratio or bucket size cannot be set.");
        return false;
    }
    if (bucket_ratio && !(0.0 < bucket_ratio && bucket_ratio <= 1.0)) {
        LOGF_ERR("Cannot bucketize weight regions. Bucket ratio must be positive real number smaller than or equal to 1 .");
        return false;
    }
    if (!_regions.has_weight_regions()) {
        LOGF_ERR("Cannot bucketize weight regions. Visis does not weight regions. Call SetWeightRegions.");
        return false;
    }
    LOGF_INF("======== Bucketing weight regions.");
    Clock clock;
    _regions.BucketizeWeightRegions(bucket_ratio, bucket_size);
    double time = clock.TimeInSeconds();
    data::PTreeGeom rec;
    rec.Put("bucket_ratio_in", bucket_ratio ? *bucket_ratio : std::numeric_limits<double>::quiet_NaN());
    rec.Put("bucket_size_in", bucket_size ? *bucket_size : std::numeric_limits<double>::quiet_NaN());
    rec.Put("time", time);
    _records.PutChild("weight_regions_bucketing", rec);
    LOGF_INF("==DONE== Bucketing weight regions. | Time " << time << " s.");
    return true;
}

bool Visis::SetRouteProperties(
    bool cyclic,
    std::optional<trivis::geom::FPoint> anchor_0,
    std::optional<trivis::geom::FPoint> anchor_n
) {
    if (anchor_0 && anchor_n && cyclic) {
        LOGF_ERR("Cannot set start and end anchors and a cyclic route. Double-anchored route must by always non-cyclic.");
        return false;
    }
    if (anchor_0 || anchor_n) {
        if (!_has_vis) {
            LOGF_ERR("Cannot set anchors. Visis does not have a Vis instance. Call InitVis.");
            return false;
        }
        for (const auto &loc: _guards) {
            if (anchor_0 && loc.is_anchor_0) {
                LOGF_ERR("Cannot set start anchor. Guards already have one!");
                return false;
            }
            if (anchor_n && loc.is_anchor_n) {
                LOGF_ERR("Cannot set end anchor. Guards already have one!");
                return false;
            }
        }
        if (anchor_0) {
            const auto &anchor = anchor_0;
            if (!_vis.pl().FindTriangle(*anchor, _vis.triangles(), {})) {
                LOGF_ERR("Cannot set start anchor " << anchor->ToString() << ". The anchor point is outside the map.");
                return false;
            }
            _guards.push_back({*anchor, true, false});
            _guards_flags.push_back(-1);
        }
        if (anchor_n) {
            const auto &anchor = anchor_n;
            if (!_vis.pl().FindTriangle(*anchor, _vis.triangles(), {})) {
                LOGF_ERR("Cannot set end anchor " << anchor->ToString() << ". The anchor point is outside the map.");
                return false;
            }
            _guards.push_back({*anchor, false, true});
            _guards_flags.push_back(-2);
        }
        if (anchor_0 || anchor_n) {
            int n = static_cast<int>(_guards.size());
            auto vis_regions = guards::ComputeVisibilityRegions(_guards, _vis, _vis_model);
            _coverage.clear();
            _coverage.reserve(n);
            for (int i = 0; i < n; ++i) {
                auto &reg = vis_regions[i].region;
                const auto &loc = _guards[i];
                auto cov_reg = coverage::MakeVisRegionWithApprox(_regions.map().limits(), std::move(reg), _vis_model.sample_arc_edges_angle(), loc.is_anchor_0, loc.is_anchor_n);
                _coverage.push_back(std::move(cov_reg));
            }
        }
    }
    _records.Put("route_cyclic", cyclic);
    if (anchor_0) _records.PutGeom("route_anchor_0", *anchor_0);
    if (anchor_n) _records.PutGeom("route_anchor_n", *anchor_n);
    _route_cyclic = cyclic;
    _route_anchor_0 = std::move(anchor_0);
    _route_anchor_n = std::move(anchor_n);
    return true;
}

// utility function
std::vector<double> MakeVec(
    int max_size,
    const std::function<bool(int)> &is_ok,
    const std::function<double(int)> &get_value
) {
    std::vector<double> vec;
    vec.reserve(max_size);
    for (int i = 0; i < max_size; ++i) if (is_ok(i)) vec.push_back(get_value(i));
    return vec;
}

// utility function
inline PTree Stats(const std::vector<double> &metric) {
    return ToPTree(statistics::ComputeVectorStatistics(metric));
}

// utility variable
const std::map<std::string, guards::MergeTrianglesSelectionRule> merge_triangles_selection_rule_map = {
    {"first", guards::MergeTrianglesSelectionRule::kFirst},
    {"best", guards::MergeTrianglesSelectionRule::kBest}
};

// utility variable
const std::map<std::string, guards::MergeTrianglesOrdering> merge_triangles_ordering_map = {
    {"default", guards::MergeTrianglesOrdering::kDefault},
    {"small", guards::MergeTrianglesOrdering::kSmall},
    {"large", guards::MergeTrianglesOrdering::kLarge},
    {"ears", guards::MergeTrianglesOrdering::kEars},
    {"ears_small", guards::MergeTrianglesOrdering::kEarsSmall},
    {"ears_large", guards::MergeTrianglesOrdering::kEarsLarge},
    {"open", guards::MergeTrianglesOrdering::kOpen},
    {"open_small", guards::MergeTrianglesOrdering::kOpenSmall},
    {"open_large", guards::MergeTrianglesOrdering::kOpenLarge}
};

template<typename T>
void PutParam(
    data::PTreeGeom &records,
    const std::optional<T> &param,
    const std::string &param_name
) {
    if (param) {
        records.Put(param_name, *param);
    }
}

bool Visis::GenerateGuards(
    const std::string &method,
    const Visis::GuardsParam &param
) {
    if (!_has_vis) {
        LOGF_ERR("Cannot generate guards. Visis does not have a Vis instance. Call InitVis.");
        return false;
    }
    if (!_regions.has_target_regions()) {
        LOGF_ERR("Cannot generate guards. Visis has no target regions. Call SetTargetRegions.");
        return false;
    }
    LOGF_INF("======== Generating guards. Method: '" << method << "'.");
    int n_orig = static_cast<int>(_guards.size());

    Clock clock, clock_total;
    guards::Guards guards_temp;
    std::optional<Clipper2Lib::Paths64> uncovered_region_temp = std::nullopt;
    const std::string cov_prefix = "cov-";
    bool unknown_method = false;
    if (method == "random") {

        if (!param.n_max) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param n_max was not set!");
            return false;
        }
        if (!param.random_seed) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param random_seed was not set!");
            return false;
        }
        guards_temp = guards::GuardsRandom(_vis, *param.n_max, *param.random_seed);

    } else if (method == "reflex") {

        if (!param.reflex_tolerance) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param reflex_tolerance was not set!");
            return false;
        }
        if (!param.reflex_dist_from_vertex) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param reflex_dist_from_vertex was not set!");
            return false;
        }
        if (_vis_model.robustness_radius().has_value()) {
            double dist_from_vertex = param.reflex_dist_from_vertex.value() + _vis_model.robustness_radius().value() + 1e-6;
            guards_temp = guards::GuardsReflex(_vis, *param.reflex_tolerance, dist_from_vertex, param.n_max, param.random_seed);
        } else {
            guards_temp = guards::GuardsReflex(_vis, *param.reflex_tolerance, param.reflex_dist_from_vertex.value(), param.n_max, param.random_seed);
        }

    } else if (method == "ccdt") {

        if (_vis_model.robustness_radius().has_value()) {
            LOGF_WRN("Method '" << method << "' does not implement robust radius. The value will be ignored.");
        }

        geom::FPolygons triangles;
        guards_temp = guards::GuardsCCDT(_vis, _vis_model.vis_radius(), param.n_max, param.random_seed, &triangles);

    } else if (method == "ka") {

        if (!param.ka_ordering) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param ka_ordering was not set!");
            return false;
        }
        if (!param.ka_selection_rule) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param ka_selection_rule was not set!");
            return false;
        }
        auto ordering_lower = boost::algorithm::to_lower_copy(*param.ka_ordering);
        auto o_it = merge_triangles_ordering_map.find(ordering_lower);
        if (o_it == merge_triangles_ordering_map.end()) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Unknown ordering: '" << *param.ka_ordering << "'.");
            return false;
        }
        auto selection_rule_lower = boost::algorithm::to_lower_copy(*param.ka_selection_rule);
        auto sr_it = merge_triangles_selection_rule_map.find(selection_rule_lower);
        if (sr_it == merge_triangles_selection_rule_map.end()) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Unknown selection rule: '" << *param.ka_selection_rule << "'.");
            return false;
        }
        auto convex_partitions = guards::MergeTrianglesToConvexPartitions(_vis, o_it->second, sr_it->second);
        auto convex_polygons = guards::ToPolygons(convex_partitions, _vis.mesh());
        trivis::geom::FPolygons guarded_convex_polygons;
        guards_temp = guards::GuardsKA(convex_polygons, _vis, _vis_model, param.n_max, param.random_seed, &guarded_convex_polygons);

    } else if (method.rfind(cov_prefix, 0) != std::string::npos) {

        if (!param.random_seed) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param random_seed was not set!");
            return false;
        }
        if (!param.min_coverage_ratio) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param min_coverage_ratio was not set!");
            return false;
        }
        if (!(0.0 < param.min_coverage_ratio && param.min_coverage_ratio < 1.0)) {
            LOGF_ERR("Cannot generate guards with method '" << method << "'. Param min_coverage_ratio must be positive real number smaller than 1!");
            return false;
        }

        uncovered_region_temp = coverage::Difference(_regions.target_regions_c(), _coverage);

        auto method_suffix = method;
        method_suffix.erase(0, cov_prefix.size());

        if (method_suffix == "rand") {

            coverage::SampleCoverageRandom(
                _coverage,
                _guards,
                uncovered_region_temp.value(),
                _vis,
                _regions.target_regions_c_area(),
                _vis_model,
                param.n_max,
                param.min_coverage_ratio.value(),
                param.random_seed.value());

        } else if (method_suffix == "inf") {

            coverage::SampleCoverageInformed(
                _coverage,
                _guards,
                uncovered_region_temp.value(),
                _vis,
                _regions.target_regions_c_area(),
                _vis_model,
                param.n_max,
                param.min_coverage_ratio.value(),
                param.random_seed.value());

        } else if (method_suffix == "dual") {

            if (!param.n_dual_samples && !param.n_dual_samples_per_square_unit) {
                LOGF_ERR("Cannot generate guards with method '" << method << "'. Param n_dual_samples or n_dual_samples_per_square_unit were not set!");
                return false;
            }

            if (param.n_dual_samples && param.n_dual_samples == 0) {
                LOGF_ERR("Cannot generate guards with method '" << method << "'. Param n_dual_samples must not be zero!");
                return false;
            }

            if (param.n_dual_samples_per_square_unit && param.n_dual_samples_per_square_unit <= 0.0) {
                LOGF_ERR("Cannot generate guards with method '" << method << "'. Param n_dual_samples_per_square_unit must be positive!");
                return false;
            }

            coverage::SampleCoverageInformedDual(
                _coverage,
                _guards,
                uncovered_region_temp.value(),
                _vis,
                _regions.target_regions_c_area(),
                _vis_model,
                param.n_max,
                param.min_coverage_ratio.value(),
                param.random_seed.value(),
                param.n_dual_samples,
                param.n_dual_samples_per_square_unit);

        } else {
            unknown_method = true;
        }
    } else {
        unknown_method = true;
    }
    if (unknown_method) {
        LOGF_ERR("Cannot generate guards with method '" << method << "'. The method does not exist!");
        return false;
    }

    double time_generate = clock.TimeInSeconds();

    data::PTreeGeom rec_vis_reg_stats;
    double time_postprocess = 0.0;
    if (!guards_temp.empty()) {
        clock.Restart();
        LOGF_INF("Removing duplicate guards.");
        guards::RemoveDuplicates(guards_temp);
        LOGF_INF("Computing visibility regions.");
        auto vis_regions_with_statistics = guards::ComputeVisibilityRegions(guards_temp, _vis, _vis_model);
        int n = static_cast<int>(vis_regions_with_statistics.size());
        LOGF_INF("Converting the regions to approximate coverage.");
        coverage::Coverage coverage_temp;
        coverage_temp.reserve(n);
        for (int i = 0; i < n; ++i) {
            auto reg = std::move(vis_regions_with_statistics[i].region);
            const auto &guard = guards_temp[i];
            auto cov_reg = coverage::MakeVisRegionWithApprox(_regions.map().limits(), std::move(reg), _vis_model.sample_arc_edges_angle(), guard.is_anchor_0, guard.is_anchor_n);
            coverage_temp.push_back(std::move(cov_reg));
        }
        time_postprocess = clock.TimeInSeconds();
        if (param.record_vis_reg_stats && !vis_regions_with_statistics.empty()) {
            const auto &vr = vis_regions_with_statistics;
            auto stats_vis_reg_time = Stats(MakeVec(n, [](int i) { return true; }, [vr](int i) -> double { return vr[i].time; }));
            auto stats_vis_reg_num_exp = Stats(MakeVec(n, [](int i) { return true; }, [vr](int i) -> double { return vr[i].stats.num_expansions; }));
            auto stats_vis_reg_max_depth = Stats(MakeVec(n, [](int i) { return true; }, [vr](int i) -> double { return vr[i].stats.max_recursion_depth; }));
            const auto &cov = coverage_temp;
            auto stats_vis_reg_n_vertices = Stats(MakeVec(n, [](int i) { return true; }, [cov](int i) -> double { return static_cast<double>(cov[i].orig.vertices.size()); }));
            auto stats_vis_reg_area = Stats(MakeVec(n, [](int i) { return true; }, [cov](int i) -> double { return cov[i].orig.Area(); }));
            auto stats_appx_n_vertices = Stats(MakeVec(n, [](int i) { return true; }, [cov](int i) -> double { return static_cast<double>(cov[i].approx.polygon.size()); }));
            auto stats_appx_area = Stats(MakeVec(n, [](int i) { return true; }, [cov](int i) -> double { return geom::Area(cov[i].approx.polygon); }));
            rec_vis_reg_stats.PutChild("vis_reg_time", stats_vis_reg_time);
            rec_vis_reg_stats.PutChild("vis_reg_num_exp", stats_vis_reg_num_exp);
            rec_vis_reg_stats.PutChild("vis_reg_max_depth", stats_vis_reg_max_depth);
            rec_vis_reg_stats.PutChild("vis_reg_n_vertices", stats_vis_reg_n_vertices);
            rec_vis_reg_stats.PutChild("vis_reg_area", stats_vis_reg_area);
            rec_vis_reg_stats.PutChild("vis_reg_appx_n_vertices", stats_appx_n_vertices);
            rec_vis_reg_stats.PutChild("vis_reg_appx_area", stats_appx_area);
        }
        std::move(guards_temp.begin(), guards_temp.end(), std::back_inserter(_guards));
        std::move(coverage_temp.begin(), coverage_temp.end(), std::back_inserter(_coverage));
    }

    data::PTreeGeom rec_coverage;
    if (param.record_coverage) {
        if (!uncovered_region_temp) {
            uncovered_region_temp = coverage::Difference(_regions.target_regions_c(), _coverage);
        }
        double covered_ratio = 1.0 - Clipper2Lib::Area(*uncovered_region_temp) / _regions.target_regions_c_area();
        double left_to_cover = std::max(0.0, _regions.min_coverage_ratio() - covered_ratio);
        double uncovered_area = uncovered_region_temp->empty() ? 0.0 : geom::Area(utils::FromClipper(*uncovered_region_temp, _regions.map().limits()));
        rec_coverage.Put("coverage_ratio", _regions.min_coverage_ratio());
        rec_coverage.Put("covered_ratio", covered_ratio);
        rec_coverage.Put("left_to_cover", left_to_cover);
        rec_coverage.Put("uncovered_area", uncovered_area);
    }

    ++_guards_calls_cnt;
    int n_new = static_cast<int>(_guards.size());
    int n_diff = n_new - n_orig;
    _guards_flags.insert(_guards_flags.end(), n_diff, _guards_calls_cnt);
    LOGF_INF("Generated " << n_diff << " new guards.");

    double time_total = clock_total.TimeInSeconds();
    double time_rest = time_total - (time_generate + time_postprocess);

    data::PTreeGeom rec;
    rec.Put("method", method);
    data::PTreeGeom rec_param;
    PutParam(rec_param, param.n_max, "n_max");
    PutParam(rec_param, param.random_seed, "random_seed");
    PutParam(rec_param, param.reflex_tolerance, "reflex_tolerance");
    PutParam(rec_param, param.reflex_dist_from_vertex, "reflex_dist_from_vertex");
    PutParam(rec_param, param.ka_ordering, "ka_ordering");
    PutParam(rec_param, param.ka_selection_rule, "ka_selection_rule");
    PutParam(rec_param, param.min_coverage_ratio, "min_coverage_ratio");
    PutParam(rec_param, param.n_dual_samples, "n_dual_samples");
    PutParam(rec_param, param.n_dual_samples_per_square_unit, "n_dual_samples_per_square_unit");
    rec.PutChild("param", rec_param);
    rec.Put("n_orig", n_orig);
    rec.Put("n_new", n_new);
    rec.Put("n_diff", n_diff);
    rec.Put("time_generate", time_generate);
    rec.Put("time_postprocess", time_postprocess);
    rec.Put("time_rest", time_rest);
    rec.Put("time_total", time_total);
    if (param.record_vis_reg_stats) rec.PutChild("vis_reg_stats", rec_vis_reg_stats);
    if (param.record_coverage) rec.PutChild("coverage", rec_coverage);
    _records.PutChild("guards_" + std::to_string(_guards_calls_cnt), rec);
    _has_guards = true;
    LOGF_INF("==DONE== Generating guards. Method: " << method << " | Time " << time_total << " s.");
    return true;
}

bool Visis::ProcessCoverage(
    bool filter
) {
    if (!_regions.has_target_regions()) {
        LOGF_ERR("Cannot process coverage. Visis has no target regions. Call SetTargetRegions.");
        return false;
    }
    if (!_regions.has_target_buckets()) {
        LOGF_ERR("Cannot process coverage. Visis has no target buckets. Call BucketizeTargetRegions.");
        return false;
    }
    if (!_has_guards) {
        LOGF_ERR("Cannot process coverage. Visis has no guards. Call GenerateGuards.");
        return false;
    }
    if (_coverage.size() != _guards.size()) {
        LOGF_ERR("Cannot process coverage. The coverage has a different size from the list of guards. This should never happen.");
        return false;
    }
    LOGF_INF("======== Processing coverage.");
    Clock clock;
    data::PTreeGeom rec_filter;
    double left_to_cover;
    double covered_ratio;
    int n_prior_filtering = static_cast<int>(_coverage.size());
    if (filter) {
        coverage::FilteringStatistics stat;
        _filtered_indices = coverage::FilterCoverage(
            _coverage,
            _regions.target_regions_c(),
            _regions.target_regions_c_area(),
            _regions.target_buckets(),
            _regions.min_coverage_ratio(),
            &stat,
            &_uncovered_region);
        rec_filter.Put("n_prior_filtering", stat.n_prior_filtering);
        rec_filter.Put("n_after_filtering", stat.n_after_filtering);
        rec_filter.Put("n_filtered", stat.n_filtered);
        rec_filter.Put("regions_n_buckets_avg", stat.regions_n_buckets_avg);
        rec_filter.Put("time_bucketing_coverage_avg", stat.time_bucketing_coverage_avg);
        rec_filter.Put("time_filtering_avg", stat.time_filtering_avg);
        rec_filter.Put("time_bucketing_coverage", stat.time_bucketing_coverage);
        rec_filter.Put("time_filtering", stat.time_filtering);
        left_to_cover = stat.left_to_cover;
        covered_ratio = stat.covered_ratio;
    } else {
        _filtered_indices = utils::Range(n_prior_filtering);
        _uncovered_region = coverage::Difference(_regions.target_regions_c(), _coverage);
        covered_ratio = 1.0 - Clipper2Lib::Area(_uncovered_region) / _regions.target_regions_c_area();
        left_to_cover = std::max(0.0, _regions.min_coverage_ratio() - covered_ratio);
    }
    // Sort filtered indices according to distance from origin (this can speed up Dijkstra algorithm that is run later).
    // Anchors are the exception. Init anchor stays in front, while the exit anchor is moved to the back.
    if (_route_anchor_0 && _route_anchor_n) {
        std::swap(_filtered_indices[1], _filtered_indices.back());
    } else if (_route_anchor_n) {
        std::swap(_filtered_indices[0], _filtered_indices.back());
    }
    std::sort(
        _filtered_indices.begin() + static_cast<int>(_route_anchor_0.has_value()),
        _filtered_indices.end() - static_cast<int>(_route_anchor_n.has_value()),
        [this](int i, int j) { return _coverage[i].orig.seed.Norm() < _coverage[j].orig.seed.Norm(); }
    );
    double time_total = clock.TimeInSeconds();
    int n_after_filtering = static_cast<int>(_filtered_indices.size());
    int n_filtered = n_prior_filtering - n_after_filtering;
    LOGF_INF("Filtered out " << n_filtered << " regions.");
    data::PTreeGeom rec;
    rec.PutChild("filtering", rec_filter);
    rec.Put("filtered", filter);
    rec.Put("n_prior_filtering", n_prior_filtering);
    rec.Put("n_after_filtering", n_after_filtering);
    rec.Put("n_filtered", n_filtered);
    rec.Put("coverage_ratio", _regions.min_coverage_ratio());
    rec.Put("covered_ratio", covered_ratio);
    rec.Put("left_to_cover", left_to_cover);
    rec.Put("uncovered_area", geom::Area(utils::FromClipper(_uncovered_region, _regions.map().limits())));
    rec.Put("time", time_total);
    _records.PutChild("process_coverage", rec);
    _has_coverage = true;
    LOGF_INF("==DONE== Processing coverage. | Time " << time_total << " s.");
    return true;
}

bool Visis::ApplyFilter() {
    if (!_has_coverage) {
        LOGF_ERR("Cannot apply filter. Visis does not have a coverage. Call ProcessCoverage.");
        return false;
    }
    LOGF_INF("======== Applying filter.");
    Clock clock;
    _guards = utils::Select(_guards, _filtered_indices);
    _guards_flags = utils::Select(_guards_flags, _filtered_indices);
    _coverage = utils::Select(_coverage, _filtered_indices);
    _filtered_indices.clear();
    double time_total = clock.TimeInSeconds();
    _records.Put("apply_filter_time", time_total);
    LOGF_INF("==DONE== Applying filter. | Time " << time_total << " s.");
    return true;
}

bool Visis::ConstructCitiesVisibilityGraph() {
    if (!_path_finder.has_vis_graph_reflex()) {
        LOGF_ERR("Cannot construct the cities visibility graph. Visis does not have the reflex visibility graph. Call ConstructVisibilityGraphReflex.");
        return false;
    }
    if (!_has_coverage) {
        LOGF_ERR("Cannot construct the cities visibility graph. Visis does not have a coverage. Call ProcessCoverage.");
        return false;
    }
    LOGF_INF("======== Constructing the cities visibility graph.");
    Clock clock;

    // Get the cities.
    int n_cities = static_cast<int>(_coverage.size());
    geom::FPoints cities(n_cities);
    for (int i = 0; i < n_cities; ++i) {
        cities[i] = _coverage[i].orig.seed;
    }
    double time_get_cities = clock.TimeInSeconds();

    // Compute the cities visibility graph.
    auto rec_opt = _path_finder.ConstructCitiesVisibilityGraph(_vis, cities);
    if (!rec_opt) {
        LOGF_ERR("Path finder could not construct the cities visibility graph.");
        return false;
    }

    _records.Put("time_get_cities", time_get_cities);
    _records.PutChild("vis_graph_cities", *rec_opt);
    LOGF_INF("==DONE== Constructing the cities visibility graph. | Time " << clock.TimeInSeconds() << " s.");
    return true;
}

bool Visis::PrecomputeCitiesShortestPaths() {
    if (!_path_finder.has_vis_graph_cities()) {
        LOGF_ERR("Cannot compute the cities shortest paths. Visis does not have a cities visibility graph. Call ConstructVisibilityGraphCities.");
        return false;
    }
    LOGF_INF("======== Computing the cities shortest paths.");
    Clock clock;
    auto rec_opt = _path_finder.PrecomputeCitiesShortestPaths();
    if (!rec_opt) {
        LOGF_ERR("Path finder could not be precompute the cities shortest paths.");
        return false;
    }
    _records.PutChild("shortest_paths_cities", *rec_opt);
    LOGF_INF("==DONE== Computing the cities shortest paths. | Time " << clock.TimeInSeconds() << " s.");
    return true;
}

bool Visis::InitDrawer(
    double resolution,
    double relative_frame_width
) {
    if (!_regions.has_map()) {
        LOGF_ERR("Cannot initialize Drawer. Visis does not have a map. Call LoadMap.");
        return false;
    }
    LOGF_INF("Initializing drawer.");
    _drawer = dr::MakeMapDrawer(_regions.map(), resolution, relative_frame_width);
    _has_drawer = true;
    return true;
}

void Visis::DrawMap(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw map. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing map to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawMesh(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw mesh. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_has_vis) {
        LOGF_ERR("Cannot draw mesh. Visis does not have a Vis instance. Call InitVis.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing mesh to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    dr::FancyDrawMesh(_drawer, _vis);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawTargetRegions(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw target regions. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_regions.has_target_regions()) {
        LOGF_ERR("Cannot draw target regions. Visis does not have them. Call SetTargetRegions.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing target regions to " << path << ".");
    auto reg_of_int = utils::FromClipper(_regions.target_regions_c(), _regions.map().limits());
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap(dr::kColorGray);
    _drawer.DrawPolygons(reg_of_int, 0.05, dr::kColorBlack, dr::kColorWhite);
    if (_regions.has_weight_regions()) {
        double w_max = *std::max_element(_regions.weights().begin(), _regions.weights().end());
        for (int i = 0; i < _regions.n_weight_regions(); ++i) {
            auto w_reg = utils::FromClipper(_regions.weight_regions_c()[i], _regions.map().limits());
            double w = _regions.weights()[i];
            _drawer.DrawPolygons(w_reg, dr::kColorLimeGreen, w / w_max);
            _drawer.DrawPolygons(w_reg, 0.05, dr::kColorBlack);
        }
    }
    _drawer.DrawHoles();
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawTargetBuckets(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw target buckets. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_regions.has_target_buckets()) {
        LOGF_ERR("Cannot draw target buckets. Visis does not have them. Call BucketizeTargetRegions.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing target buckets to " << path << ".");
    auto colors = dr::RandomColors(static_cast<int>(_regions.target_buckets().size()));
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i = 0; i < _regions.target_buckets().size(); ++i) {
        _drawer.DrawPolygons(utils::FromClipper(_regions.target_buckets()[i].region, _regions.map().limits()), colors[i]);
    }
    _drawer.DrawHoles();
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawGuards(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw guards. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_has_guards) {
        LOGF_ERR("Cannot draw guards. Call GenerateGuards[...] (at least once).");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing guards to " << path << ".");
    int n = static_cast<int>(_guards.size());
    int n_filtered = static_cast<int>(_filtered_indices.size());
    std::vector<int> indices;
    std::vector<int> indices_filtered_out;
    if (_filtered_indices.empty()) {
        indices = utils::Range(n);
    } else {
        indices = _filtered_indices;
        auto aux = std::vector<bool>(n, false);
        for (int i: indices) aux[i] = true;
        for (int i = 0; i < n; ++i) if (!aux[i]) indices_filtered_out.push_back(i);
    }
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i: indices_filtered_out) {
        const auto &l = _guards[i];
        _drawer.DrawPoint(l.point, 0.500, dr::kColorDeepSkyBlue, 0.25);
        _drawer.DrawPoint(l.point, 0.050, dr::kColorDeepSkyBlue, 0.25);
        _drawer.DrawPoint(l.point, 0.005, dr::kColorDeepSkyBlue, 0.25);
        _drawer.DrawText(std::to_string(_guards_flags[i]), l.point, 0.5, dr::kColorBlack, 0.2);
    }
    for (int i: indices) {
        const auto &l = _guards[i];
        _drawer.DrawPoint(l.point, 0.500, dr::kColorRed, 0.50);
        _drawer.DrawPoint(l.point, 0.050, dr::kColorRed, 0.50);
        _drawer.DrawPoint(l.point, 0.005, dr::kColorRed, 0.50);
        _drawer.DrawText(std::to_string(_guards_flags[i]), l.point, 0.5, dr::kColorBlack, 0.2);
    }
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawCoverage(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw coverage. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_has_coverage) {
        LOGF_ERR("Cannot draw coverage. Visis does not have a coverage. Call ProcessCoverage.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing coverage to " << path << ".");
    int n = static_cast<int>(_coverage.size());
    int n_filtered = static_cast<int>(_filtered_indices.size());
    std::vector<int> indices;
    std::vector<int> indices_filtered_out;
    if (_filtered_indices.empty()) {
        indices = utils::Range(n);
    } else {
        indices = _filtered_indices;
        auto aux = std::vector<bool>(n, false);
        for (int i: indices) aux[i] = true;
        for (int i = 0; i < n; ++i) if (!aux[i]) indices_filtered_out.push_back(i);
    }
    auto colors = dr::RandomColors(n);
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i: indices) {
        FancyDrawRadialVisibilityRegion(_drawer, _coverage[i].orig, colors[i], 1.0, 1.0);
    }
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawUncoveredRegion(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw uncovered region. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_has_coverage) {
        LOGF_ERR("Cannot draw uncovered region. Visis does not have a coverage. Call ProcessGuards.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing uncovered region to " << path << ".");
    int n = static_cast<int>(_coverage.size());
    int n_filtered = static_cast<int>(_filtered_indices.size());
    std::vector<int> indices;
    std::vector<int> indices_filtered_out;
    if (_filtered_indices.empty()) {
        indices = utils::Range(n);
    } else {
        indices = _filtered_indices;
        auto aux = std::vector<bool>(n, false);
        for (int i: indices) aux[i] = true;
        for (int i = 0; i < n; ++i) if (!aux[i]) indices_filtered_out.push_back(i);
    }
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap(dr::kColorWhite, dr::kColorLightGray);
    _drawer.DrawPolygons(utils::FromClipper(_uncovered_region, _regions.map().limits()), dr::kColorRed, 0.75);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawReflexVisibilityGraph(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw the reflex visibility graph. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_path_finder.has_vis_graph_reflex()) {
        LOGF_ERR("Cannot draw the reflex visibility graph. Visis does not have it. Call ConstructReflexVisibilityGraph.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing the reflex visibility graph to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i = 0; i < _path_finder.n_reflex(); ++i) {
        const auto &pi = _path_finder.reflex_points()[i];
        const auto &visible_vertices = _path_finder.VisibleReflexFromReflex(i);
        for (int j: visible_vertices) {
            const auto &pj = _path_finder.reflex_points()[j];
            _drawer.DrawLine(pi, pj, 0.05, dr::kColorRed, 0.25);
        }
    }
    _drawer.DrawPoints(_path_finder.reflex_points(), 0.2, dr::kColorBlack);
    _drawer.DrawPoints(_path_finder.reflex_points(), 0.1, dr::kColorRed);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawCitiesVisibilityGraph(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw the cities visibility graph. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_path_finder.has_vis_graph_cities()) {
        LOGF_ERR("Cannot draw the cities visibility graph. Visis does not have it. Call ConstructCitiesVisibilityGraph.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing visibility graph to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i = 0; i < _path_finder.n_cities(); ++i) {
        const auto &pi = _path_finder.cities()[i];
        const auto &visible_vertices = _path_finder.VisibleReflexFromCity(i);
        for (int reflex_vertex_id: visible_vertices) {
            const auto &pj = _path_finder.reflex_points()[reflex_vertex_id];
            _drawer.DrawLine(pi, pj, 0.05, dr::kColorBlue, 0.25);
        }
    }
    for (int i = 0; i < _path_finder.n_cities(); ++i) {
        const auto &pi = _path_finder.cities()[i];
        const auto &visible_cities = _path_finder.VisibleCitiesFromCity(i);
        for (int j: visible_cities) {
            const auto &pj = _path_finder.cities()[j];
            _drawer.DrawLine(pi, pj, 0.05, dr::kColorLime, 0.25);
        }
    }
    _drawer.DrawPoints(_path_finder.cities(), 0.2, dr::kColorBlack);
    _drawer.DrawPoints(_path_finder.cities(), 0.1, dr::kColorBlue);
    _drawer.DrawPoints(_path_finder.reflex_points(), 0.2, dr::kColorBlack);
    _drawer.DrawPoints(_path_finder.reflex_points(), 0.1, dr::kColorRed);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawReflexShortestPaths(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw the reflex shortest paths. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_path_finder.has_vis_graph_reflex()) {
        LOGF_ERR("Cannot draw the reflex shortest paths. Visis does not have the reflex visibility graph. Call ConstructReflexVisibilityGraph.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing the reflex shortest paths to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i = 0; i < _path_finder.n_reflex(); ++i) {
        for (int j = 0; j < _path_finder.n_reflex(); ++j) {
            trivis::geom::FPoints point_path;
            _path_finder.ShortestPathReflex(_vis, i, j, &point_path);
            _drawer.DrawPath(point_path, 0.05, dr::kColorRed, 0.25);
        }
    }
    _drawer.DrawPoints(_path_finder.reflex_points(), 0.2, dr::kColorBlack);
    _drawer.DrawPoints(_path_finder.reflex_points(), 0.1, dr::kColorRed);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawCitiesShortestPaths(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw the cities shortest paths. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_path_finder.has_vis_graph_cities()) {
        LOGF_ERR("Cannot draw the cities shortest paths. Visis does not have the cities visibility graph. Call ConstructCitiesVisibilityGraph.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing the cities shortest paths to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    for (int i = 0; i < _path_finder.n_cities(); ++i) {
        for (int j = 0; j < _path_finder.n_cities(); ++j) {
            trivis::geom::FPoints point_path;
            _path_finder.ShortestPathCities(_vis, i, j, &point_path);
            _drawer.DrawPath(point_path, 0.05, dr::kColorBlue, 0.25);
        }
    }
    _drawer.DrawPoints(_path_finder.cities(), 0.2, dr::kColorBlack);
    _drawer.DrawPoints(_path_finder.cities(), 0.1, dr::kColorBlue);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::DrawCitiesPath(const std::string &file) {
    if (!_has_drawer) {
        LOGF_ERR("Cannot draw the cities path. Visis does not have a drawer. Call InitDrawer.");
        return;
    }
    if (!_has_cities_path) {
        LOGF_ERR("Cannot draw the cities path. Visis does not have the cities path. Call SolveDiscreteSearch.");
        return;
    }
    fs::path path(file.c_str());
    if (!exists(path.parent_path())) fs::create_directories(path.parent_path());
    bool png = path.extension().string() == ".png";
    if (!png) path = fs::path(path).replace_extension(".pdf").string();
    LOGF_INF("Drawing cities path to " << path << ".");
    if (png) _drawer.OpenImage(); else _drawer.OpenPDF(path.string());
    _drawer.DrawMap();
    dr::FancyDrawPath(_drawer, _cities_reflex_point_path, _path_finder.n_reflex(), _cities_reflex_id_path, 0.5, 0.5);
    if (png) _drawer.SaveToPng(path.string());
    _drawer.Close();
}

void Visis::RecordMapProperties() {
    if (!_regions.has_map()) {
        LOGF_ERR("Cannot record map properties. Visis does not have a map. Call LoadMap.");
        return;
    }
    LOGF_INF("======== Recording map properties");
    Clock clock;
    const auto &map = _regions.map();
    data::PTreeGeom rec;
    rec.Put("n_borders", 1);
    rec.Put("n_holes", map.holes().size());
    rec.Put("n_points", map.ToPoints().size());
    rec.Put("lim_x_min", map.limits().x_min);
    rec.Put("lim_y_min", map.limits().y_min);
    rec.Put("lim_x_max", map.limits().x_max);
    rec.Put("lim_y_max", map.limits().y_max);
    rec.Put("area", geom::Area(map.border()) + geom::Area(map.holes()));
    double time = clock.TimeInSeconds();
    _records.PutChild("map_info", rec);
    LOGF_INF("==DONE== Recording map properties | Time " << time << " s.");
}

void Visis::RecordVisProperties(int n_random_points) {
    if (!_has_vis) {
        LOGF_ERR("Cannot record Vis properties. Visis does not have a Vis instance. Call InitVis.");
        return;
    }
    LOGF_INF("======== Recording Vis properties");
    Clock clock;
    const auto &pl = _vis.pl();
    const auto &mesh = _vis.mesh();
    data::PTreeGeom rec;
    rec.Put("visibility_radius", _vis_model.vis_radius().has_value() ? _vis_model.vis_radius().value() : std::numeric_limits<double>::infinity());
    rec.Put("bucketing_n_buckets", pl.buckets().data().size());
    rec.Put("bucketing_n_col", pl.n_col());
    rec.Put("bucketing_n_row", pl.n_row());
    rec.Put("bucketing_x_scale", pl.x_scale());
    rec.Put("bucketing_y_scale", pl.y_scale());
    rec.Put("mesh_n_vertices", mesh.vertices.size());
    rec.Put("mesh_n_edges", mesh.edges.size());
    rec.Put("mesh_n_triangles", mesh.triangles.size());
    std::vector<double> free_edges_len;
    std::vector<double> obstacle_edges_len;
    for (const auto &e: mesh.edges) {
        double length = mesh.point(e.vertices[0]).DistanceTo(mesh.point(e.vertices[1]));
        if (e.is_boundary()) {
            obstacle_edges_len.push_back(length);
        } else {
            free_edges_len.push_back(length);
        }
    }
    rec.PutChild("mesh_edge_length_obstacle", Stats(free_edges_len));
    rec.PutChild("mesh_edge_length_free", Stats(obstacle_edges_len));
    rec.PutChild("mesh_edge_length_all", Stats(utils::Concatenate(free_edges_len, obstacle_edges_len)));
    std::vector<double> triangle_areas;
    for (const auto &triangle: _vis.triangles()) {
        triangle_areas.push_back(geom::Area(triangle));
    }
    rec.PutChild("mesh_triangle_area", Stats(triangle_areas));
    {
        // Compute statistics regarding visibility regions computed from mesh vertices.
        auto metrics = statistics::EvaluateVisibilityRegionComputations(_vis, _vis_model.vis_radius());
        const auto &mm = metrics;
        int m = static_cast<int>(metrics.size());
        rec.PutChild("reg_vertex_triangle_found", Stats(MakeVec(m, [mm](int i) { return true; }, [mm](int i) -> double { return mm[i].tri_found; })));
        rec.PutChild("reg_vertex_time_bucketing", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_bucketing; })));
        rec.PutChild("reg_vertex_time_tea", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_tea; })));
        rec.PutChild("reg_vertex_n_expansions", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].n_expansions; })));
        rec.PutChild("reg_vertex_max_rec_dep", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].max_recursion_depth; })));
        rec.PutChild("reg_vertex_time_intersections", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_intersections; })));
        rec.PutChild("reg_vertex_time_antennas", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_antennas; })));
        rec.PutChild("reg_vertex_n_vertices", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].n_vertices; })));
        rec.PutChild("reg_vertex_had_antennas", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].had_antennas; })));
        rec.PutChild("reg_vertex_time_circ_intersect", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].time_circ_intersect; })));
        rec.PutChild("reg_vertex_is_valid", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].is_valid; })));
        rec.PutChild("reg_vertex_area", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].area; })));
        rec.PutChild("reg_vertex_free_edges_total_len", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].free_edges_total_len; })));
        rec.PutChild("reg_vertex_obst_edges_total_len", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].obst_edges_total_len; })));
        rec.PutChild("reg_vertex_time_total", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].time_total; })));
    }
    if (n_random_points > 0) {
        // Compute statistics regarding visibility regions computed from 1 million random points.
        geom::FPoints random_points;
        std::mt19937 rng{42};
        for (int i = 0; i < n_random_points; ++i) {
            random_points.push_back(utils::UniformRandomPointInRandomTriangle(_vis.triangles(), rng));
        }
        auto metrics = statistics::EvaluateVisibilityRegionComputations(_vis, _vis_model.vis_radius(), random_points);
        const auto &mm = metrics;
        int m = static_cast<int>(metrics.size());
        rec.PutChild("reg_rand_triangle_found", Stats(MakeVec(m, [mm](int i) { return true; }, [mm](int i) -> double { return mm[i].tri_found; })));
        rec.PutChild("reg_rand_time_bucketing", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_bucketing; })));
        rec.PutChild("reg_rand_time_tea", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_tea; })));
        rec.PutChild("reg_rand_n_expansions", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].n_expansions; })));
        rec.PutChild("reg_rand_max_rec_dep", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].max_recursion_depth; })));
        rec.PutChild("reg_rand_time_intersections", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_intersections; })));
        rec.PutChild("reg_rand_time_antennas", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].time_antennas; })));
        rec.PutChild("reg_rand_n_vertices", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].n_vertices; })));
        rec.PutChild("reg_rand_had_antennas", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].had_antennas; })));
        rec.PutChild("reg_rand_time_circ_intersect", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].time_circ_intersect; })));
        rec.PutChild("reg_rand_is_valid", Stats(MakeVec(m, [mm](int i) { return mm[i].tri_found; }, [mm](int i) -> double { return mm[i].is_valid; })));
        rec.PutChild("reg_rand_area", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].area; })));
        rec.PutChild("reg_rand_free_edges_total_len", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].free_edges_total_len; })));
        rec.PutChild("reg_rand_obst_edges_total_len", Stats(MakeVec(m, [mm](int i) { return mm[i].is_valid; }, [mm](int i) -> double { return mm[i].obst_edges_total_len; })));
        rec.PutChild("reg_rand_time_total", Stats(MakeVec(m, [mm](int i) { return true; }, [mm](int i) -> double { return mm[i].time_total; })));
    }
    double time = clock.TimeInSeconds();
    _records.PutChild("vis_info", rec);
    LOGF_INF("==DONE== Recording Vis properties | Time " << time << " s.");
}

void Visis::PrintRecords() const {
    std::stringstream ss;
    boost::property_tree::json_parser::write_json(ss, _records.get());
    LOGF_INF("Printing records:\n" << ss.str());
}

bool Visis::WriteRecords(const std::string &file) const {
    try {
        _records.Write(file);
    } catch (const std::exception &e) {
        LOGF_ERR("Could not write to file " << file << ".");
        return false;
    }
    return true;
}

void Visis::TestPathfinding() {
    if (!_has_coverage) {
        LOGF_ERR("Cannot test pathfinding. Visis does not have a coverage. Call ProcessGuards.");
        return;
    }
    LOGF_INF("======== Testing pathfinding");
    Clock clock_total;
    Clock clock;

    LOGF_INF("Getting the cities.");
    // Get the cities.
    int n_cities = static_cast<int>(_filtered_indices.size());
    geom::FPoints cities(n_cities);
    for (int i = 0; i < n_cities; ++i) {
        cities[i] = _coverage[_filtered_indices[i]].orig.seed;
    }

    LOGF_INF("Creating PathFinder instances.");
    pathfinding::PathFinder pf;
    pf.ConstructReflexVisibilityGraph(_vis);

    pathfinding::PathFinder pf_pre_reflex = pf;
    pf_pre_reflex.PrecomputeReflexShortestPaths();

    pathfinding::PathFinder pf_cities = pf;
    pf_cities.ConstructCitiesVisibilityGraph(_vis, cities);

    pathfinding::PathFinder pf_cities_pre_reflex = pf_pre_reflex;
    pf_cities_pre_reflex.ConstructCitiesVisibilityGraph(_vis, cities);

    pathfinding::PathFinder pf_cities_pre_cities = pf_cities;
    pf_cities_pre_cities.PrecomputeCitiesShortestPaths();

    LOGF_INF("Testing pathfinding: reflex.");
    { // Test the reflex-shortest paths.
        LOGF_INF("Testing pathfinding: reflex -- pf.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf = std::vector<std::vector<double>>(pf.n_reflex(), std::vector<double>(pf.n_reflex(), -1.0));
        for (int i = 0; i < pf.n_reflex(); ++i) {
            for (int j = 0; j < pf.n_reflex(); ++j) {
                len_pf[i][j] = pf.ShortestPathReflex(_vis, i, j);
            }
        }
        double time_pf = clock.TimeInSeconds();

        LOGF_INF("Testing pathfinding: reflex -- pf_pre_reflex.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf_pre_reflex = std::vector<std::vector<double>>(pf.n_reflex(), std::vector<double>(pf.n_reflex(), -1.0));
        for (int i = 0; i < pf.n_reflex(); ++i) {
            for (int j = 0; j < pf.n_reflex(); ++j) {
                len_pf_pre_reflex[i][j] = pf_pre_reflex.ShortestPathReflex(_vis, i, j);
            }
        }
        double time_pf_pre_reflex = clock.TimeInSeconds();

        for (int i = 0; i < pf.n_reflex(); ++i) {
            for (int j = 0; j < pf.n_reflex(); ++j) {
                double diff = std::abs(len_pf[i][j] - len_pf_pre_reflex[i][j]);
                if (diff > 1e-12) {
                    LOGF_ERR("Reflex: pf and pf_pre_reflex give different lengths for " << i << " and " << j << ". Abs diff: " << diff << ".");
                }
            }
        }

        LOGF_INF("Reflex: pf_pre_reflex is " << time_pf / time_pf_pre_reflex << "-times faster than pf.");
    }

    LOGF_INF("Testing pathfinding: cities.");
    { // Test the cities-shortest paths.
        LOGF_INF("Testing pathfinding: cities -- pf_cities.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf_cities = std::vector<std::vector<double>>(n_cities, std::vector<double>(n_cities, -1.0));
        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                len_pf_cities[i][j] = pf_cities.ShortestPathCities(_vis, i, j);
            }
        }
        double time_pf_cities = clock.TimeInSeconds();

        LOGF_INF("Testing pathfinding: cities -- pf_cities_pre_reflex.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf_cities_pre_reflex = std::vector<std::vector<double>>(n_cities, std::vector<double>(n_cities, -1.0));
        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                len_pf_cities_pre_reflex[i][j] = pf_cities_pre_reflex.ShortestPathCities(_vis, i, j);
            }
        }
        double time_pf_cities_pre_reflex = clock.TimeInSeconds();

        LOGF_INF("Testing pathfinding: cities -- pf_cities_pre_cities.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf_cities_pre_cities = std::vector<std::vector<double>>(n_cities, std::vector<double>(n_cities, -1.0));
        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                len_pf_cities_pre_cities[i][j] = pf_cities_pre_cities.ShortestPathCities(_vis, i, j);
            }
        }
        double time_pf_cities_pre_cities = clock.TimeInSeconds();

        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                double diff = std::abs(len_pf_cities[i][j] - len_pf_cities_pre_reflex[i][j]);
                if (diff > 1e-12) {
                    LOGF_ERR("Points: pf_cities and pf_cities_pre_reflex give different lengths for " << i << " and " << j << ". Abs diff: " << diff << ".");
                }
                double diff2 = std::abs(len_pf_cities[i][j] - len_pf_cities_pre_cities[i][j]);
                if (diff > 1e-12) {
                    LOGF_ERR("Points: pf_cities and pf_cities_pre_cities give different lengths for " << i << " and " << j << ". Abs diff: " << diff << ".");
                }
            }
        }

        LOGF_INF("Points: pf_cities_pre_reflex is " << time_pf_cities / time_pf_cities_pre_reflex << "-times faster than pf_cities.");
        LOGF_INF("Points: pf_cities_pre_cities is " << time_pf_cities / time_pf_cities_pre_cities << "-times faster than pf_cities.");
        LOGF_INF("Points: pf_cities_pre_cities is " << time_pf_cities_pre_reflex / time_pf_cities_pre_cities << "-times faster than pf_cities_pre_reflex.");
    }

    LOGF_INF("Testing pathfinding: points.");
    { // Test the points-shortest paths.
        LOGF_INF("Testing pathfinding: points -- pf.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf = std::vector<std::vector<double>>(n_cities, std::vector<double>(n_cities, -1.0));
        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                len_pf[i][j] = pf.ShortestPathPoints(_vis, cities[i], cities[j]);
            }
        }
        double time_pf = clock.TimeInSeconds();

        LOGF_INF("Testing pathfinding: points -- pf_pre_reflex.");
        clock.Restart();
        std::vector<std::vector<double>> len_pf_pre_reflex = std::vector<std::vector<double>>(n_cities, std::vector<double>(n_cities, -1.0));
        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                len_pf_pre_reflex[i][j] = pf_pre_reflex.ShortestPathPoints(_vis, cities[i], cities[j]);
            }
        }
        double time_pf_pre_reflex = clock.TimeInSeconds();

        for (int i = 0; i < n_cities; ++i) {
            for (int j = 0; j < n_cities; ++j) {
                double diff = std::abs(len_pf[i][j] - len_pf_pre_reflex[i][j]);
                if (diff > 1e-12) {
                    LOGF_ERR("Points: pf and pf_pre_reflex give different lengths for " << i << " and " << j << ". Abs diff: " << diff << ".");
                }
            }
        }

        LOGF_INF("Points: pf_pre_reflex is " << time_pf / time_pf_pre_reflex << "-times faster than pf.");
    }

    double time_total = clock_total.TimeInSeconds();
    LOGF_INF("==DONE== Testing pathfinding | Time " << time_total << " s.");
}
