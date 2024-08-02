/**
 * File:   vis_region_approx.cc
 *
 * Date:   31.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/coverage/approx_vis_region.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::coverage;

using namespace trivis;

ApproxVisRegion coverage::MakeApproxVisRegion(
    const geom::FLimits &lim,
    const RadialVisibilityRegion &vis_reg,
    const std::optional<double> &max_sample_beta
) {
    ApproxVisRegion ret;
    ret.arcs = vis_reg;
    if (vis_reg.radius.has_value() && vis_reg.radius.value() > 0.0) {
        ret.max_sample_beta = max_sample_beta.value();
        ret.arcs.SampleArcEdges(ret.max_sample_beta);
    }
    ret.clipper = utils::ToClipper(ret.arcs, lim);
    ret.polygon = utils::FromClipper(ret.clipper, lim);
    return ret;
}

VisRegionWithApprox coverage::MakeVisRegionWithApprox(
    const geom::FLimits &lim,
    RadialVisibilityRegion vis_reg,
    const std::optional<double> &max_sample_beta,
    bool is_anchor_0,
    bool is_anchor_n
) {
    VisRegionWithApprox ret;
    ret.is_anchor_0 = is_anchor_0;
    ret.is_anchor_n = is_anchor_n;
    ret.approx = MakeApproxVisRegion(lim, vis_reg, max_sample_beta);
    ret.orig = std::move(vis_reg);
    return ret;
}

Clipper2Lib::Paths64 coverage::Difference(
    const Clipper2Lib::Paths64 &subject,
    const VisRegionWithApprox &clip
) {
    Clipper2Lib::Paths64 ret;
    Clipper2Lib::Clipper64 clipper;
    clipper.AddSubject(subject);
    clipper.AddClip({clip.approx.clipper});
    clipper.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, ret);
    return ret;
}
