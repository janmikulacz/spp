/**
 * File:   approx_vis_region.h
 *
 * Date:   31.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_COVERAGE_APPROX_VIS_REGION_H_
#define VISIS_CORE_COVERAGE_APPROX_VIS_REGION_H_

#include "trivis/trivis.h"

#include <optional>

namespace visis::core::coverage {

struct ApproxVisRegion {
    // Parameters used when converting arcs to poly-lines.
    double max_sample_beta;
    // The same as the original, but arcs are converted to poly-lines.
    trivis::RadialVisibilityRegion arcs;
    // The above converted to clipper representation and simplified.
    Clipper2Lib::Path64 clipper;
    // The above converted to floating-point polygon.
    trivis::geom::FPolygon polygon;
};

struct VisRegionWithApprox {
    // Original visibility region.
    trivis::RadialVisibilityRegion orig;
    // Visibility region's approximations.
    ApproxVisRegion approx;
    // Flag if the region is the start anchor.
    bool is_anchor_0 = false;
    // Flag if the region is the end anchor.
    bool is_anchor_n = false;
};

ApproxVisRegion MakeApproxVisRegion(
    const trivis::geom::FLimits &lim,
    const trivis::RadialVisibilityRegion &vis_reg,
    const std::optional<double> &max_sample_beta = M_PI / 36.0
);

VisRegionWithApprox MakeVisRegionWithApprox(
    const trivis::geom::FLimits &lim,
    trivis::RadialVisibilityRegion vis_reg,
    const std::optional<double> &max_sample_beta = M_PI / 36.0,
    bool is_anchor_0 = false,
    bool is_anchor_n = false
);

Clipper2Lib::Paths64 Difference(
    const Clipper2Lib::Paths64 &subject,
    const VisRegionWithApprox &clip
);

}

#endif //VISIS_CORE_COVERAGE_APPROX_VIS_REGION_H_
