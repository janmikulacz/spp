/**
 * File:   target_regions.cc
 *
 * Date:   31.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/coverage/target_regions.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::coverage;

using namespace trivis;

void TargetRegions::SetMap(geom::PolyMap map) {
    _map = std::move(map);
    _map_c = utils::ToClipper(_map);
    _map_c_lim = ComputeClipperLimits(_map_c);
    _map_c_area = Clipper2Lib::Area(_map_c);
    _has_map = true;
}

void TargetRegions::SetTargetRegions(
    double min_coverage_ratio,
    std::optional<std::vector<geom::FPolygons>> target_regions
) {
    if (!target_regions) {
        geom::FPolygons reg;
        reg.push_back(_map.border());
        for (const auto &hole: _map.holes()) {
            reg.push_back(hole);
        }
        target_regions = std::vector<geom::FPolygons>{};
        target_regions->emplace_back(std::move(reg));
    }
    Clipper2Lib::Clipper64 clipper;
    for (const auto &region: *target_regions) {
        clipper.AddSubject(utils::ToClipper(region, _map.limits()));
    }
    Clipper2Lib::Paths64 target_regions_c;
    clipper.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, target_regions_c);
    clipper.Clear();
    clipper.AddSubject(target_regions_c);
    clipper.AddClip(_map_c);
    clipper.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, target_regions_c);
    _min_coverage_ratio = min_coverage_ratio;
    _target_regions = std::move(*target_regions);
    _target_regions_c = std::move(target_regions_c);
    _target_regions_c_area = Clipper2Lib::Area(_target_regions_c);
    _has_target_regions = true;
}

void TargetRegions::SetWeightRegions(
    std::optional<std::vector<geom::FPolygons>> weight_regions,
    std::optional<std::vector<double>> weights
) {
    std::vector<Clipper2Lib::Paths64> weight_regions_c;
    std::vector<double> weight_regions_c_areas;
    if (weight_regions) {
        weight_regions_c.reserve(weight_regions->size());
        weight_regions_c_areas.reserve(weight_regions->size());
        Clipper2Lib::Clipper64 clipper;
        for (const auto &reg: *weight_regions) {
            clipper.Clear();
            clipper.AddSubject(utils::ToClipper(reg, _map.limits()));
            Clipper2Lib::Paths64 reg_c;
            clipper.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, reg_c);
            clipper.Clear();
            clipper.AddSubject(reg_c);
            clipper.AddClip(_target_regions_c);
            clipper.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, reg_c);
            weight_regions_c_areas.push_back(Clipper2Lib::Area(reg_c));
            weight_regions_c.push_back(std::move(reg_c));
        }
    } else {
        weight_regions = _target_regions;
        weight_regions_c = std::vector<Clipper2Lib::Paths64>{};
        weight_regions_c.emplace_back(_target_regions_c);
        weight_regions_c_areas.push_back(_target_regions_c_area);
    }
    if (!weights) {
        weights = std::vector<double>(weight_regions->size(), 1.0);
    }
    _weight_regions = std::move(*weight_regions);
    _weight_regions_c = std::move(weight_regions_c);
    _weight_regions_c_areas = std::move(weight_regions_c_areas);
    _weights = std::move(*weights);
    _n_weight_regions = static_cast<int>(_weight_regions_c.size());
    _total_weight_c = 0.0;
    for (int i = 0; i < _n_weight_regions; ++i) {
        _total_weight_c += _weights[i] * _weight_regions_c_areas[i];
    }
    _has_weight_regions = true;
}

int64_t ComputeBucketSize(
    std::optional<double> bucket_ratio,
    std::optional<double> bucket_size,
    const coverage::ClipperLimits &lim_c,
    const coverage::ClipperLimits &map_lim_c,
    const geom::FLimits &map_lim
) {
    if (bucket_ratio) {
        double map_max_dim = static_cast<double>(std::max(map_lim_c.x_max - map_lim_c.x_min, map_lim_c.y_max - map_lim_c.y_min));
        double reg_of_int_max_dim = static_cast<double>(std::max(lim_c.x_max - lim_c.x_min, lim_c.y_max - lim_c.y_min));
        return static_cast<int64_t>(std::ceil(reg_of_int_max_dim / std::ceil(reg_of_int_max_dim / (*bucket_ratio * map_max_dim))));
    } else { // bucket_size
        return utils::ToClipper(*bucket_size, map_lim);
    }
}

void TargetRegions::BucketizeTargetRegions(
    std::optional<double> bucket_ratio,
    std::optional<double> bucket_size
) {
    auto lim_c = coverage::ComputeClipperLimits(_target_regions_c);
    _target_bucket_size_c = ComputeBucketSize(bucket_ratio, bucket_size, lim_c, _map_c_lim, _map.limits());
    _target_buckets = coverage::MakeBuckets(_target_regions_c, lim_c, _target_bucket_size_c);
    _has_target_buckets = true;
}

void TargetRegions::BucketizeWeightRegions(
    std::optional<double> bucket_ratio,
    std::optional<double> bucket_size
) {
    _weight_bucket_sizes_c.resize(_n_weight_regions);
    _weight_buckets.resize(_n_weight_regions);
    for (int i = 0; i < _n_weight_regions; ++i) {
        double probability_weight = _weights[i] / _total_weight_c;
        const auto lim_c = coverage::ComputeClipperLimits(_weight_regions_c[i]);
        _weight_bucket_sizes_c[i] = ComputeBucketSize(bucket_ratio, bucket_size, lim_c, _map_c_lim, _map.limits());
        _weight_buckets[i] = coverage::MakeBuckets(_weight_regions_c[i], lim_c, _weight_bucket_sizes_c[i], probability_weight);
    }
    _has_weight_buckets = true;
}
