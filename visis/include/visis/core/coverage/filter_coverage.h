/**
 * File:   filter_coverage.h
 *
 * Date:   24.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_COVERAGE_FILTER_COVERAGE_H_
#define VISIS_CORE_COVERAGE_FILTER_COVERAGE_H_

#include "visis/core/coverage/coverage.h"

#include "visis/core/coverage/buckets.h"

namespace visis::core::coverage {

struct FilteringStatistics {
    int n_prior_filtering = 0;
    int n_after_filtering = 0;
    int n_filtered = 0;
    double regions_n_buckets_avg = 0.0;
    double covered_ratio = 0.0;
    double left_to_cover = 0.0;
    double time_bucketing_coverage = 0.0;
    double time_bucketing_coverage_avg = 0.0;
    double time_filtering = 0.0;
    double time_filtering_avg = 0.0;
};

std::vector<int> FilterCoverage(
    const Coverage &coverage,
    const Clipper2Lib::Paths64 &target_regions_c,
    double target_regions_c_area,
    const Buckets &target_buckets,
    double min_coverage_ratio = .999,
    FilteringStatistics *stat_out = nullptr,
    Clipper2Lib::Paths64 *uncovered_region_out = nullptr
);

Coverage FilterCoverageLight(
    const Coverage &coverage,
    const Clipper2Lib::Paths64 &target_regions_c,
    double min_coverage_ratio = .999
);

Coverage FilterRedundantRegions(
    const Coverage &coverage
);

}

#endif //VISIS_CORE_COVERAGE_FILTER_COVERAGE_H_
