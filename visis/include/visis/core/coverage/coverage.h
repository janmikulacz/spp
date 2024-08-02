/**
 * File:   coverage.h
 *
 * Date:   24.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_COVERAGE_COVERAGE_H_
#define VISIS_CORE_COVERAGE_COVERAGE_H_

#include "visis/core/coverage/approx_vis_region.h"

namespace visis::core::coverage {

using Coverage = std::vector<coverage::VisRegionWithApprox>;

Clipper2Lib::Paths64 Difference(
    const Clipper2Lib::Paths64 &subject,
    const Coverage &clip,
    int cluster_size = 100
);

}

#endif //VISIS_CORE_COVERAGE_COVERAGE_H_
