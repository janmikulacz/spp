/**
 * File:   bucket.h
 *
 * Date:   24.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_COVERAGE_BUCKET_H_
#define VISIS_CORE_COVERAGE_BUCKET_H_

#include "trivis/trivis.h"

namespace visis::core::coverage {

struct ClipperLimits {
    int64_t x_min = std::numeric_limits<int64_t>::max();
    int64_t y_min = std::numeric_limits<int64_t>::max();
    int64_t x_max = std::numeric_limits<int64_t>::lowest();
    int64_t y_max = std::numeric_limits<int64_t>::lowest();
};

struct Bucket {
    int id;
    int x;
    int y;
    double weight;
    ClipperLimits lim;
    Clipper2Lib::Rect64 rect;
    Clipper2Lib::Paths64 region;
    Clipper2Lib::Rect64 region_bounding_box;
    double region_area;
    bool region_is_rect;
};

using Buckets = std::vector<Bucket>;

ClipperLimits ComputeClipperLimits(const Clipper2Lib::Path64 &path);

ClipperLimits ComputeClipperLimits(const Clipper2Lib::Paths64 &paths);

coverage::Bucket MakeBucket(
    int id,
    int x,
    int y,
    const coverage::ClipperLimits &lim,
    const Clipper2Lib::Paths64 &uncovered_region,
    double weight = 0.0
);

coverage::Buckets MakeBuckets(
    const Clipper2Lib::Paths64 &regions,
    const coverage::ClipperLimits &lim,
    int64_t bucket_size_c,
    double weight = 0.0
);

}

#endif //VISIS_CORE_COVERAGE_BUCKET_H_
