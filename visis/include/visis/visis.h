/**
 * File:   visis.h
 *
 * Date:   11.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_VISIS_H_
#define VISIS_VISIS_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>

#include "visis/core/coverage/coverage.h"
#include "visis/core/coverage/target_regions.h"
#include "visis/core/pathfinding/path_finder.h"
#include "visis/core/guards/guard.h"
#include "visis/core/guards/reflex_vertex.h"
#include "visis/core/visibility/visibility_model.h"
#include "visis/data/ptree_geom.h"
#include "visis/log/log.h"
#include "visis/drawing/drawing.h"

#include "trivis/trivis.h"

namespace visis {

class Visis {
public:

    struct GuardsParam {
        std::optional<unsigned> n_max;
        std::optional<unsigned> random_seed;
        std::optional<double> reflex_tolerance;
        std::optional<double> reflex_dist_from_vertex;
        std::optional<std::string> ka_ordering;
        std::optional<std::string> ka_selection_rule;
        std::optional<double> min_coverage_ratio;
        std::optional<unsigned> n_dual_samples;
        std::optional<double> n_dual_samples_per_square_unit;
        bool record_vis_reg_stats = false;
        bool record_coverage = false;
    };

    bool SetVisibilityModel(
        std::optional<double> vis_radius = std::nullopt,
        std::optional<double> robustness_radius = std::nullopt,
        std::optional<double> robustness_sample_dist = std::nullopt,
        std::optional<double> sample_arc_edges_angle = std::nullopt,
        std::optional<double> sample_arc_edges_dist = std::nullopt
    );

    bool SetRobotRadius(double radius);

    bool LoadMap(
        const std::string &map_path_or_name,
        const std::optional<std::string> &map_ext = std::nullopt,
        const std::optional<std::string> &map_dir = std::nullopt,
        const std::optional<double> &map_scale = std::nullopt
    );

    bool InitVis(
        double bucket_size = 1.0,
        bool optimize_buckets = false
    );

    bool ConstructReflexVisibilityGraph();

    bool PrecomputeReflexShortestPaths();

    bool SetTargetRegions(
        double min_coverage_ratio = .999,
        std::optional<std::vector<trivis::geom::FPolygons>> target_regions = std::nullopt
    );

    bool SetWeightRegions(
        std::optional<std::vector<trivis::geom::FPolygons>> weight_regions = std::nullopt,
        std::optional<std::vector<double>> weights = std::nullopt
    );

    bool BucketizeTargetRegions(
        std::optional<double> bucket_ratio = 0.1,
        std::optional<double> bucket_size = std::nullopt
    );

    bool BucketizeWeightRegions(
        std::optional<double> bucket_ratio = 0.1,
        std::optional<double> bucket_size = std::nullopt
    );

    bool SetRouteProperties(
        bool cyclic = true,
        std::optional<trivis::geom::FPoint> anchor_0 = std::nullopt,
        std::optional<trivis::geom::FPoint> anchor_n = std::nullopt
    );

    bool GenerateGuards(
        const std::string &method,
        const GuardsParam &param
    );

    bool ProcessCoverage(
        bool filter = false
    );

    bool ApplyFilter();

    bool ConstructCitiesVisibilityGraph();

    bool PrecomputeCitiesShortestPaths();

    bool InitDrawer(
        double resolution = .025,
        double relative_frame_width = 0.02
    );

    void RecordMapProperties();

    void RecordVisProperties(int n_random_points);

    void PrintRecords() const;

    bool WriteRecords(const std::string &file) const;

    void DrawMap(const std::string &file);

    void DrawMesh(const std::string &file);

    void DrawTargetRegions(const std::string &file);

    void DrawTargetBuckets(const std::string &file);

    void DrawGuards(const std::string &file);

    void DrawCoverage(const std::string &file);

    void DrawUncoveredRegion(const std::string &file);

    void DrawReflexVisibilityGraph(const std::string &file);

    void DrawCitiesVisibilityGraph(const std::string &file);

    void DrawReflexShortestPaths(const std::string &file);

    void DrawCitiesShortestPaths(const std::string &file);

    void DrawCitiesPath(const std::string &file);

    void TestPathfinding();

    [[nodiscard]] const auto &records() const { return _records; }

    [[nodiscard]] auto &records_writable() { return _records; }

private:

    std::optional<double> _rob_radius;

    core::visibility::VisibilityModel _vis_model;

    trivis::Trivis _vis;
    bool _has_vis = false;

    bool _route_cyclic = true;
    std::optional<trivis::geom::FPoint> _route_anchor_0;
    std::optional<trivis::geom::FPoint> _route_anchor_n;

    core::coverage::TargetRegions _regions;

    core::guards::Guards _guards;
    std::vector<int> _guards_flags;
    int _guards_calls_cnt = 0;
    bool _has_guards = false;

    core::coverage::Coverage _coverage;
    Clipper2Lib::Paths64 _uncovered_region;
    std::vector<int> _filtered_indices;
    bool _has_coverage = false;

    core::pathfinding::PathFinder _path_finder;

    std::vector<int> _cities_id_path;
    std::vector<int> _cities_reflex_id_path;
    trivis::geom::FPoints _cities_reflex_point_path;
    bool _has_cities_path;

    data::PTreeGeom _records;

    drawing::MapDrawer _drawer = drawing::MakeMapDrawer();
    bool _has_drawer = false;
};

}

#endif //VISIS_VISIS_H_
