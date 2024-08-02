/**
 * File:   target_regions.h
 *
 * Date:   31.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_COVERAGE_TARGET_REGIONS_H_
#define VISIS_CORE_COVERAGE_TARGET_REGIONS_H_

#include "visis/core/coverage/buckets.h"

#include <optional>

#include "trivis/trivis.h"

namespace visis::core::coverage {

class TargetRegions {
public:

    void SetMap(
        trivis::geom::PolyMap map
    );

    void SetTargetRegions(
        double min_coverage_ratio,
        std::optional<std::vector<trivis::geom::FPolygons>> target_regions = std::nullopt
    );

    void SetWeightRegions(
        std::optional<std::vector<trivis::geom::FPolygons>> weight_regions = std::nullopt,
        std::optional<std::vector<double>> weights = std::nullopt
    );

    void BucketizeTargetRegions(
        std::optional<double> bucket_ratio = std::nullopt,
        std::optional<double> bucket_size = std::nullopt
    );

    void BucketizeWeightRegions(
        std::optional<double> bucket_ratio = std::nullopt,
        std::optional<double> bucket_size = std::nullopt
    );

    [[nodiscard]] const auto &map() const { return _map; }
    [[nodiscard]] const auto &map_c() const { return _map_c; }
    [[nodiscard]] const auto &map_c_lim() const { return _map_c_lim; }
    [[nodiscard]] double map_c_area() const { return _map_c_area; }
    [[nodiscard]] bool has_map() const { return _has_map; }

    [[nodiscard]] double min_coverage_ratio() const { return _min_coverage_ratio; }
    [[nodiscard]] const auto &target_regions() const { return _target_regions; }
    [[nodiscard]] const auto &target_regions_c() const { return _target_regions_c; }
    [[nodiscard]] double target_regions_c_area() const { return _target_regions_c_area; }
    [[nodiscard]] bool has_target_regions() const { return _has_target_regions; }

    [[nodiscard]] int n_weight_regions() const { return _n_weight_regions; }
    [[nodiscard]] const auto &weight_regions() const { return _weight_regions; }
    [[nodiscard]] const auto &weight_regions_c() const { return _weight_regions_c; }
    [[nodiscard]] const auto &weight_regions_c_areas() const { return _weight_regions_c_areas; }
    [[nodiscard]] const auto &weights() const { return _weights; }
    [[nodiscard]] double total_weight_c() const { return _total_weight_c; }
    [[nodiscard]] bool has_weight_regions() const { return _has_weight_regions; }

    [[nodiscard]] const auto &target_buckets() const { return _target_buckets; }
    [[nodiscard]] int64_t target_bucket_size_c() const { return _target_bucket_size_c; }
    [[nodiscard]] bool has_target_buckets() const { return _has_target_buckets; }

    [[nodiscard]] const auto &weight_buckets() const { return _weight_buckets; }
    [[nodiscard]] const auto &weight_bucket_sizes_c() const { return _weight_bucket_sizes_c; }
    [[nodiscard]] bool has_weight_buckets() const { return _has_weight_buckets; }

private:

    trivis::geom::PolyMap _map;
    Clipper2Lib::Paths64 _map_c;
    ClipperLimits _map_c_lim;
    double _map_c_area = 0.0;
    bool _has_map = false;

    double _min_coverage_ratio = 0.0;
    std::vector<trivis::geom::FPolygons> _target_regions;
    Clipper2Lib::Paths64 _target_regions_c;
    double _target_regions_c_area = 0.0;
    bool _has_target_regions = false;

    int _n_weight_regions = 0;
    std::vector<trivis::geom::FPolygons> _weight_regions;
    std::vector<Clipper2Lib::Paths64> _weight_regions_c;
    std::vector<double> _weight_regions_c_areas;
    std::vector<double> _weights;
    double _total_weight_c;
    bool _has_weight_regions = false;

    int64_t _target_bucket_size_c;
    core::coverage::Buckets _target_buckets;
    bool _has_target_buckets = false;

    std::vector<int64_t> _weight_bucket_sizes_c;
    std::vector<core::coverage::Buckets> _weight_buckets;
    bool _has_weight_buckets = false;

};

}

#endif //VISIS_CORE_COVERAGE_TARGET_REGIONS_H_
