/**
 * File:   guard.h
 *
 * Date:   20.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_GUARD_H_
#define VISIS_CORE_GUARDS_GUARD_H_

#include "trivis/trivis.h"

#include <optional>

namespace visis::core::guards {

struct Guard {
    trivis::geom::FPoint point;
    bool is_anchor_0 = false;
    bool is_anchor_n = false;
    std::optional<int> id_v = std::nullopt;
    std::optional<int> id_e = std::nullopt;
    std::optional<int> id_t = std::nullopt;
};

using Guards = std::vector<Guard>;

}

#endif //VISIS_CORE_GUARDS_GUARD_H_
