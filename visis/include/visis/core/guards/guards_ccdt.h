/**
 * File:   guards_ccdt.h
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_GUARDS_CCDT_H_
#define VISIS_CORE_GUARDS_GUARDS_CCDT_H_

#include "visis/core/guards/guard.h"

#include "trivis/trivis.h"

#include <random>

namespace visis::core::guards {

[[nodiscard]] Guards GuardsCCDT(
    const trivis::Trivis &vis,
    std::optional<double> vis_radius = std::nullopt,
    std::optional<unsigned> n_max = std::nullopt,
    std::optional<unsigned> random_seed = std::nullopt,
    trivis::geom::FPolygons *guarded_triangles_out = nullptr
);

}

#endif //VISIS_CORE_GUARDS_GUARDS_CCDT_H_
