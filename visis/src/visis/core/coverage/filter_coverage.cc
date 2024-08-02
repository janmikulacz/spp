/**
 * File:   filter_coverage.cc
 *
 * Date:   24.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/coverage/filter_coverage.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::coverage;


using Clock = trivis::utils::SimpleClock;

struct BucketIntersection {
    int bucket_id;
    Clipper2Lib::Paths64 intersection;
    Clipper2Lib::Rect64 bounding_box;
};

struct RegionBucketInfo {
    std::vector<int> local_bucket_id;
    std::vector<BucketIntersection> bucket_intersections;
    bool is_tabu = false;
    bool is_anchor = false;
};

double RegionArea(const RegionBucketInfo &region_info) {
    double area = 0.0;
    for (const auto &bucket: region_info.bucket_intersections) {
        if (!bucket.intersection.empty()) {
            area += Clipper2Lib::Area(bucket.intersection);
        }
    }
    return area;
}

std::vector<int> coverage::FilterCoverage(
    const Coverage &coverage,
    const Clipper2Lib::Paths64 &target_regions_c,
    double target_regions_c_area,
    const Buckets &target_buckets,
    double min_coverage_ratio,
    FilteringStatistics *stat_out,
    Clipper2Lib::Paths64 *uncovered_region_out
) {

    Clock clock;

    // This will be the filtered indices.
    std::vector<int> ret;

    // Merge buckets and regions into new utility structure 'regions_info'.
    clock.Restart();
    int n_regions = static_cast<int>(coverage.size());
    int n_buckets = static_cast<int>(target_buckets.size());
    int regions_n_buckets_sum = 0.0;
    std::vector<RegionBucketInfo> regions_info(n_regions, {std::vector<int>(n_buckets, -1)});
    for (int region_id = 0; region_id < n_regions; ++region_id) {
        const auto &cov_region = coverage[region_id];
        const auto &region = cov_region.approx.clipper;
        auto &region_info = regions_info[region_id];
        region_info.is_anchor = cov_region.is_anchor_0 || cov_region.is_anchor_n;
        auto region_bounding_box = Clipper2Lib::GetBounds(region);
        int region_n_buckets = 0;
        for (int bucket_id = 0; bucket_id < n_buckets; ++bucket_id) {
            const auto &bucket = target_buckets[bucket_id];
            if (region_info.is_anchor || region_bounding_box.Intersects(bucket.region_bounding_box)) {
                Clipper2Lib::Paths64 intersection;
                if (bucket.region_is_rect) {
                    intersection = Clipper2Lib::RectClip(bucket.rect, Clipper2Lib::Paths64{region});
                } else {
                    intersection = Clipper2Lib::Intersect(bucket.region, Clipper2Lib::Paths64{region}, Clipper2Lib::FillRule::NonZero);
                }
                if (region_info.is_anchor || !intersection.empty()) {
                    region_info.local_bucket_id[bucket_id] = region_n_buckets;
                    auto intersection_bb = Clipper2Lib::GetBounds(intersection);
                    region_info.bucket_intersections.push_back({bucket_id, std::move(intersection), intersection_bb});
                    ++region_n_buckets;
                }
            }
        }
        regions_n_buckets_sum += region_n_buckets;
    }
    double time_bucketing_coverage = clock.TimeInSeconds();

    // Perform the filtering.
    clock.Restart();
    auto uncovered_region = target_regions_c;
    double uncovered_ratio_goal = 1.0 - min_coverage_ratio;
    double uncovered_ratio = 1.0;
    for (int cnt = 0; uncovered_ratio > uncovered_ratio_goal && cnt < n_regions; ++cnt) {
        // Find which intersection with the regions of interest has the biggest area.
        int best_region_id = -1;
        double best_region_area = 0.0;
        for (int region_id = 0; region_id < n_regions; ++region_id) {
            auto &region_info = regions_info[region_id];
            if (region_info.is_tabu) {
                continue;
            }
            double area = RegionArea(region_info);
            if (region_info.is_anchor || area > best_region_area) {
                best_region_area = area;
                best_region_id = region_id;
                if (region_info.is_anchor) {
                    break;
                }
            }
        }
        if (best_region_id == -1) {
            break; // should not happen
        }

        auto &best_reg_info = regions_info[best_region_id];
        best_reg_info.is_tabu = true; // Make tabu because it was chosen.

        // Now update the rest of intersections by clipping away the best intersection.
        for (int region_id = 0; region_id < n_regions; ++region_id) {
            auto &region_info = regions_info[region_id];
            if (region_info.is_tabu) {
                continue;
            }
            int n_empty_buckets = 0;
            for (auto &bucket_intersection: region_info.bucket_intersections) {
                auto &intersection = bucket_intersection.intersection;
                if (intersection.empty()) {
                    ++n_empty_buckets;
                    continue;
                }
                int idx = best_reg_info.local_bucket_id[bucket_intersection.bucket_id];
                if (idx == -1) {
                    continue;
                }
                auto &intersection_bb = bucket_intersection.bounding_box;
                const auto &clip_bb = best_reg_info.bucket_intersections[idx].bounding_box;
                if (intersection_bb.Intersects(clip_bb)) {
                    const auto &clip = best_reg_info.bucket_intersections[idx].intersection;
                    auto rect_clip = Clipper2Lib::RectClip(intersection_bb, clip);
                    if (!rect_clip.empty()) {
                        intersection = Clipper2Lib::Difference(intersection, rect_clip, Clipper2Lib::FillRule::NonZero);
                        intersection_bb = Clipper2Lib::GetBounds(intersection);
                        if (intersection.empty()) {
                            ++n_empty_buckets;
                        }
                    }
                }
            }
            if (!region_info.is_anchor && n_empty_buckets >= region_info.bucket_intersections.size()) {
                region_info.is_tabu = true; // Make tabu because all buckets are empty.
            }
        }

        // Finally update stopping condition and push the selected id.
        uncovered_region = Clipper2Lib::Difference(uncovered_region, {coverage[best_region_id].approx.clipper}, Clipper2Lib::FillRule::NonZero);
        uncovered_ratio = Clipper2Lib::Area(uncovered_region) / target_regions_c_area;
        ret.push_back(best_region_id);
    }
    double time_filtering = clock.TimeInSeconds();

    // Record statistics.
    if (stat_out) {
        stat_out->n_prior_filtering = n_regions;
        stat_out->n_after_filtering = static_cast<int>(ret.size());
        stat_out->n_filtered = stat_out->n_prior_filtering - stat_out->n_after_filtering;
        stat_out->regions_n_buckets_avg = static_cast<double>(regions_n_buckets_sum) / stat_out->n_prior_filtering;
        stat_out->covered_ratio = 1.0 - uncovered_ratio;
        stat_out->left_to_cover = std::max(0.0, min_coverage_ratio - stat_out->covered_ratio);
        stat_out->time_bucketing_coverage = time_bucketing_coverage;
        stat_out->time_bucketing_coverage_avg = stat_out->time_bucketing_coverage / stat_out->n_prior_filtering;
        stat_out->time_filtering = time_filtering;
        stat_out->time_filtering_avg = stat_out->time_filtering / stat_out->n_after_filtering;
    }

    if (uncovered_region_out) {
        *uncovered_region_out = std::move(uncovered_region);
    }

    return ret;
}

Coverage coverage::FilterCoverageLight(
    const Coverage &coverage,
    const Clipper2Lib::Paths64 &target_regions_c,
    double min_coverage_ratio
) {
    // todo: use buckets
    // todo: optimize for finite visibility radius
    const int n = static_cast<int>(coverage.size());
    std::vector<double> areas(n, 0.0);
    for (int i = 0; i < n; ++i) {
        areas[i] = Clipper2Lib::Area(coverage[i].approx.clipper);
    }
    auto sorted_indices = trivis::utils::Range(n);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [areas](int i, int j) { return areas[i] < areas[j]; });
    std::vector<int> selected_indices;
    double area_to_cover = Clipper2Lib::Area(target_regions_c);
    Clipper2Lib::Clipper64 clipper;
    auto uncovered_region = target_regions_c;
    double uncovered_area = area_to_cover;
    for (int i = 0; i < n; ++i) {
        int idx = sorted_indices[i];
        clipper.AddSubject(uncovered_region);
        clipper.AddClip({coverage[idx].approx.clipper});
        clipper.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, uncovered_region);
        clipper.Clear();
        double new_uncovered_area = Clipper2Lib::Area(uncovered_region);
        if (new_uncovered_area < uncovered_area) {
            selected_indices.push_back(idx);
        }
        uncovered_area = new_uncovered_area;
        if (1.0 - uncovered_area / area_to_cover >= min_coverage_ratio) {
            break;
        }
    }
    return trivis::utils::Select(coverage, selected_indices);
}

Coverage coverage::FilterRedundantRegions(
    const Coverage &coverage
) {
    // todo: use buckets
    // todo: optimize for finite visibility radius
    const int n = static_cast<int>(coverage.size());
    std::vector<double> areas(n, 0.0);
    for (int i = 0; i < n; ++i) {
        areas[i] = Clipper2Lib::Area(coverage[i].approx.clipper);
    }
    auto sorted_indices = trivis::utils::Range(n);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [areas](int i, int j) { return areas[i] < areas[j]; });
    std::vector<int> selected_indices;
    Clipper2Lib::Clipper64 clipper;
    for (int i = 0; i < n; ++i) {
        int idx = sorted_indices[i];
        const auto &region = coverage[idx];
        auto region_remainder = Clipper2Lib::Paths64{region.approx.clipper};
        std::vector<int> current_indices = selected_indices;
        for (int j = i + 1; j < n; ++j) {
            current_indices.push_back(sorted_indices[j]);
        }
        std::vector<double> squared_dist(current_indices.size(), 0.0);
        for (int k = 0; k < current_indices.size(); ++k) {
            squared_dist[k] = region.orig.seed.SquaredDistanceTo(coverage[current_indices[k]].orig.seed);
        }
        current_indices = trivis::utils::SortBy(current_indices, squared_dist);
        for (int idx2: current_indices) {
            if (region_remainder.empty()) {
                break;
            }
            clipper.AddSubject(region_remainder);
            clipper.AddClip({coverage[idx2].approx.clipper});
            clipper.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, region_remainder);
            clipper.Clear();
        }
        if (!region_remainder.empty()) {
            selected_indices.push_back(idx);
        }
    }
    return trivis::utils::Select(coverage, selected_indices);
}
