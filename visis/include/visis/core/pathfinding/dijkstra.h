/**
 * File:   dijkstra.h
 *
 * Date:   21.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_PATHFINDING_DIJKSTRA_H_
#define VISIS_CORE_PATHFINDING_DIJKSTRA_H_

#include "visis/core/pathfinding/boost_graph.h"

namespace visis::core::pathfinding {

/// ###################################
/// # All-pairs reflex vertex queries #
/// ###################################

void ComputeShortestPathsDijkstraAllPairsReflex(
    const BoostGraph &reflex_graph,
    std::vector<std::vector<double>> &paths_lengths,
    std::vector<std::vector<int>> &paths_predecessor_map
);

std::vector<int> GetShortestIDPathAllPairsReflex(
    int source_id,
    int target_id,
    const std::vector<int> &source_predecessors
);

/// ##################
/// # 2-city queries #
/// ##################

double ComputeShortestPathDijkstraCities(
    const trivis::geom::FPoint &source_city,
    const trivis::geom::FPoint &target_city,
    bool cities_visible,
    const std::vector<int> &source_visible_reflex_vertices,
    const std::vector<int> &target_visible_reflex_vertices,
    const trivis::geom::FPoints &reflex_vertices_points,
    const std::vector<std::vector<double>> &reflex_vertices_paths_lengths,
    const std::vector<std::vector<int>> &reflex_vertices_paths_predecessor_map,
    std::vector<int> &id_path_no_endpoints
);

trivis::geom::FPoints ConvertIDPathToPointPathCities(
    const std::vector<int> &id_path_no_endpoints,
    const trivis::geom::FPoints &reflex_vertices_points,
    const trivis::geom::FPoint &source_city,
    const trivis::geom::FPoint &target_city
);

/// ##########################
/// # All-pairs city queries #
/// ##########################

void ComputeShortestPathsDijkstraAllPairsCities(
    BoostGraph reflex_graph,
    const trivis::geom::FPoints &reflex_vertices_points,
    const trivis::geom::FPoints &cities,
    const std::vector<std::vector<int>> &vis_graph_city_city,
    const std::vector<std::vector<int>> &vis_graph_city_reflex,
    std::vector<std::vector<double>> &paths_lengths,
    std::vector<std::vector<int>> &paths_predecessor_map
);

double GetShortestPathLengthAllPairsCities(
    int source_city_id,
    int target_city_id,
    int n_reflex_vertices,
    const std::vector<std::vector<double>> &paths_lengths
);

std::vector<int> GetShortestIDPathAllPairsCities(
    int source_city_id,
    int target_city_id,
    int n_reflex_vertices,
    const std::vector<std::vector<int>> &paths_predecessor_map,
    bool reversed = false
);

trivis::geom::FPoints ConvertIDPathToPointPathAllPairsCities(
    const std::vector<int> &id_path,
    const trivis::geom::FPoints &reflex_vertices_points,
    const trivis::geom::FPoints &cities
);

}

#endif //VISIS_CORE_PATHFINDING_DIJKSTRA_H_
