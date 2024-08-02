/**
 * File:   buckets.cc
 *
 * Date:   31.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/coverage/buckets.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::coverage;

ClipperLimits coverage::ComputeClipperLimits(const Clipper2Lib::Path64 &path) {
    ClipperLimits ret;
    for (const auto &p: path) {
        if (p.x < ret.x_min) ret.x_min = p.x;
        if (p.y < ret.y_min) ret.y_min = p.y;
        if (ret.x_max < p.x) ret.x_max = p.x;
        if (ret.y_max < p.y) ret.y_max = p.y;
    }
    return ret;
}
ClipperLimits coverage::ComputeClipperLimits(const Clipper2Lib::Paths64 &paths) {
    ClipperLimits ret;
    for (const auto &path: paths) {
        for (const auto &p: path) {
            if (p.x < ret.x_min) ret.x_min = p.x;
            if (p.y < ret.y_min) ret.y_min = p.y;
            if (ret.x_max < p.x) ret.x_max = p.x;
            if (ret.y_max < p.y) ret.y_max = p.y;
        }
    }
    return ret;
}

inline Clipper2Lib::Rect64 Rectangle(const coverage::ClipperLimits &lim) {
    return {lim.x_min, lim.y_min, lim.x_max, lim.y_max};
}

coverage::Bucket coverage::MakeBucket(int id, int x, int y, const ClipperLimits &lim, const Clipper2Lib::Paths64 &regions, double weight) {
    auto rect = Rectangle(lim);
    auto intersection = Clipper2Lib::RectClip(rect, regions);
    if (intersection.empty()) {
        return {id, x, y, weight, lim, rect, std::move(intersection), Clipper2Lib::Rect64{}, 0.0, false};
    }
    double intersection_area = Clipper2Lib::Area(intersection);
    bool is_rect = Clipper2Lib::Area(intersection) == static_cast<double>(rect.Width() * rect.Height());
    auto intersection_bb = Clipper2Lib::GetBounds(intersection);
    return {id, x, y, weight, lim, rect, std::move(intersection), intersection_bb, intersection_area, is_rect};
}

coverage::Buckets coverage::MakeBuckets(const Clipper2Lib::Paths64 &regions, const ClipperLimits &lim, int64_t bucket_size_c, double weight) {
    coverage::Buckets buckets;
    int bucket_id = 0;
    int rectangle_count = 0;
    int max_bucket_cnt = 0;
    int y = 0;
    for (auto y_min = lim.y_min; y_min < lim.y_max;) {
        auto y_max = y_min + bucket_size_c;
        y_max = std::min(y_max, lim.y_max);
        int x = 0;
        for (auto x_min = lim.x_min; x_min < lim.x_max;) {
            auto x_max = x_min + bucket_size_c;
            x_max = std::min(x_max, lim.x_max);
            auto bucket = MakeBucket(bucket_id, x, y, coverage::ClipperLimits{x_min, y_min, x_max, y_max}, regions, weight);
            if (bucket.region_area != 0.0) {
                if (bucket.region_is_rect) ++rectangle_count;
                buckets.push_back(std::move(bucket));
                ++bucket_id;
            }
            ++x;
            ++max_bucket_cnt;
            x_min = x_max;
        }
        ++y;
        y_min = y_max;
    }
    return buckets;
}