/**
 * File:   guards_ka.h
 *
 * Date:   28.02.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_GUARDS_KA_H_
#define VISIS_CORE_GUARDS_GUARDS_KA_H_

#include "visis/core/guards/guard.h"
#include "visis/core/visibility/visibility_model.h"

#include "visis/core/guards/merge_triangles.h"

namespace visis::core::guards {

[[nodiscard]] trivis::geom::FPoint GuardWS(
    const trivis::geom::FPolygon &convex_polygon
);

[[nodiscard]] Guards GuardsKA(
    const trivis::geom::FPolygons &convex_polygons,
    const trivis::Trivis &vis,
    const visibility::VisibilityModel &vis_model,
    std::optional<unsigned> n_max = std::nullopt,
    std::optional<unsigned> random_seed = std::nullopt,
    trivis::geom::FPolygons *guarded_convex_polygons_out = nullptr
);

}

#endif //VISIS_CORE_GUARDS_GUARDS_KA_H_
