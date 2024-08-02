/**
 * File:   sample_coverage.h
 *
 * Date:   12.06.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_COVERAGE_SAMPLE_COVERAGE_H_
#define VISIS_CORE_COVERAGE_SAMPLE_COVERAGE_H_

#include "visis/core/coverage/coverage.h"

#include "visis/core/coverage/target_regions.h"

#include "visis/core/visibility/visibility_model.h"

#include "visis/core/guards/guard.h"

#include "trivis/trivis.h"

namespace visis::core::coverage {

void SampleCoverageRandom(
    Coverage &coverage,
    guards::Guards &guards,
    Clipper2Lib::Paths64 &uncovered_region,
    const trivis::Trivis &vis,
    double target_regions_c_area,
    visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max = std::nullopt,
    double min_coverage_ratio = .999,
    unsigned random_seed = 42
);

void SampleCoverageInformed(
    Coverage &coverage,
    guards::Guards &guards,
    Clipper2Lib::Paths64 &uncovered_region,
    const trivis::Trivis &vis,
    double target_regions_c_area,
    visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max = std::nullopt,
    double min_coverage_ratio = .999,
    unsigned random_seed = 42
);

void SampleCoverageInformedDual(
    Coverage &coverage,
    guards::Guards &guards,
    Clipper2Lib::Paths64 &uncovered_region,
    const trivis::Trivis &vis,
    double target_regions_c_area,
    visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max = std::nullopt,
    double min_coverage_ratio = .999,
    unsigned random_seed = 42,
    std::optional<unsigned> n_dual_samples = 10,
    std::optional<double> n_dual_samples_per_square_unit = std::nullopt
);

}

#endif //VISIS_CORE_COVERAGE_SAMPLE_COVERAGE_H_
