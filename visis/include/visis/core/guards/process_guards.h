/**
 * File:   process_guards.h
 *
 * Date:   25.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_PROCESS_GUARDS_H_
#define VISIS_CORE_GUARDS_PROCESS_GUARDS_H_

#include "visis/core/guards/guard.h"

#include "visis/core/visibility/visibility_model.h"

#include "trivis/trivis.h"

namespace visis::core::guards {

void RemoveDuplicates(
    Guards &guards
);

struct VisRegionWithStats {
    trivis::RadialVisibilityRegion region;
    trivis::Trivis::ExpansionStats stats;
    double time = 0.0;
};

using VisibilityRegionsWithStatistics = std::vector<VisRegionWithStats>;

VisibilityRegionsWithStatistics ComputeVisibilityRegions(
    Guards &guards,
    const trivis::Trivis &vis,
    const visibility::VisibilityModel &vis_model
);

}

#endif //VISIS_CORE_GUARDS_PROCESS_GUARDS_H_
