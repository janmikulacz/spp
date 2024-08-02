/**
 * File:   guards_random.h
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_GUARDS_RANDOM_H_
#define VISIS_CORE_GUARDS_GUARDS_RANDOM_H_

#include "visis/core/guards/guard.h"

#include "trivis/trivis.h"

#include <random>

namespace visis::core::guards {

[[nodiscard]] Guards GuardsRandom(
    const trivis::Trivis &vis,
    unsigned n_max,
    unsigned random_seed = 42
);

}

#endif //VISIS_CORE_GUARDS_GUARDS_RANDOM_H_
