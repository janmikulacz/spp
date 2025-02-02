/**
 * File:   astar.h
 *
 * Date:   22.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_PATHFINDING_ASTAR_H_
#define VISIS_CORE_PATHFINDING_ASTAR_H_

#include "visis/core/pathfinding/boost_graph.h"

namespace visis::core::pathfinding {

/// ##################
/// # 2-city queries #
/// ##################

double ComputeCityPairShortestPathAstar(
    const trivis::geom::FPoint &source_city,
    const trivis::geom::FPoint &target_city,
    bool cities_visible,
    const std::vector<int> &source_visible_reflex_vertices,
    const std::vector<int> &target_visible_reflex_vertices,
    const trivis::geom::FPoints &reflex_vertices_points,
    BoostGraph &reflex_graph,
    std::vector<int> &id_path_no_cities,
    bool revert_graph = true
);

}

#endif //VISIS_CORE_PATHFINDING_ASTAR_H_
