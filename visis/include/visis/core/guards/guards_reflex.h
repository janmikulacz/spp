/**
 * File:   guards_reflex.h
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_GUARDS_REFLEX_H_
#define VISIS_CORE_GUARDS_GUARDS_REFLEX_H_

#include "visis/core/guards/guard.h"

#include "trivis/trivis.h"

namespace visis::core::guards {

[[nodiscard]] Guards GuardsReflex(
    const trivis::Trivis &vis,
    double reflex_tolerance = 0.0,
    double dist_from_vertex = 0.0,
    std::optional<unsigned> n_max = std::nullopt,
    std::optional<unsigned> random_seed = std::nullopt
);

}

#endif //VISIS_CORE_GUARDS_GUARDS_REFLEX_H_
