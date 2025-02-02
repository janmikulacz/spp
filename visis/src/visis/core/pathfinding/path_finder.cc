/**
 * File:   path_finder.cc
 *
 * Date:   27.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/pathfinding/path_finder.h"

#include "visis/core/pathfinding/dijkstra.h"

#include "visis/log/log.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::pathfinding;

using namespace trivis;

using Clock = trivis::utils::SimpleClock;
using PTree = boost::property_tree::ptree;

std::optional<data::PTreeGeom> PathFinder::ConstructReflexVisibilityGraph(const trivis::Trivis &vis) {

    _has_vis_graph_reflex = false;
    _has_shortest_paths_reflex = false;
    _has_vis_graph_cities = false;
    _has_shortest_paths_cities = false;

    Clock clock;

    // Compute reflex vertices.
    clock.Restart();
    int n_nodes = static_cast<int>(vis.mesh().vertices.size());
    _n_reflex = 0;
    _reflex_points.clear();
    _reflex_id_to_mesh_id.clear();
    _is_node_convex = std::vector<bool>(n_nodes, false);
    _mesh_id_to_reflex_id = std::vector<int>(n_nodes, -1);
    for (int node_id = 0; node_id < n_nodes; ++node_id) {
        auto neighbors = GetNeighborVertices(vis.mesh(), node_id);
        if (neighbors.size() != 2 || !IsReflex(vis.mesh(), neighbors[0], node_id, neighbors[1], 0.0)) {
            _is_node_convex.value()[node_id] = true;
            continue;
        }
        _reflex_points.push_back(vis.mesh().point(node_id));
        _reflex_id_to_mesh_id.push_back(node_id);
        _mesh_id_to_reflex_id[node_id] = _n_reflex;
        ++_n_reflex;
    }
    double time_reflex_vertices = clock.TimeInSeconds();

    // Compute the reflex-reflex visibility graph.
    clock.Restart();
    _vis_graph_reflex_reflex.resize(_n_reflex);
    { // scope of vis_graph_reflex_reflex_temp
        auto vis_graph_reflex_reflex_temp = vis.VertexVertexVisibilityGraph(std::nullopt, _is_node_convex);
        for (int ver_id = 0, reflex_ver_cnt = 0; ver_id < n_nodes; ++ver_id) {
            if (_is_node_convex.value()[ver_id]) {
                continue;
            }
            auto &visible_reflex_vertices = vis_graph_reflex_reflex_temp[ver_id];
            for (int &visible_ver_id: visible_reflex_vertices) {
                visible_ver_id = _mesh_id_to_reflex_id[visible_ver_id];
            }
            _vis_graph_reflex_reflex[reflex_ver_cnt] = std::move(visible_reflex_vertices);
            ++reflex_ver_cnt;
        }
    }
    double time_vis_graph_reflex_reflex = clock.TimeInSeconds();

    // Convert the reflex-reflex visibility graph to bool matrix.
    clock.Restart();
    _vis_graph_reflex_reflex_bool_matrix = std::vector<std::vector<bool>>(_n_reflex, std::vector<bool>(_n_reflex, false));
    for (int reflex_vertex_id_i = 0; reflex_vertex_id_i < _n_reflex; ++reflex_vertex_id_i) {
        _vis_graph_reflex_reflex_bool_matrix[reflex_vertex_id_i][reflex_vertex_id_i] = true;
        for (int reflex_vertex_id_j: _vis_graph_reflex_reflex[reflex_vertex_id_i]) {
            _vis_graph_reflex_reflex_bool_matrix[reflex_vertex_id_i][reflex_vertex_id_j] = true;
            _vis_graph_reflex_reflex_bool_matrix[reflex_vertex_id_j][reflex_vertex_id_i] = true;
        }
    }
    double time_vis_graph_reflex_reflex_to_bool = clock.TimeInSeconds();

    // Construct the reflex graph.
    clock.Restart();
    _reflex_graph = pathfinding::ConstructReflexGraph(_vis_graph_reflex_reflex, _reflex_points);
    double time_reflex_graph = clock.TimeInSeconds();

    // Initialize visibility planner.
    clock.Restart();
    _vis_planner.Init(_is_node_convex.value(), _mesh_id_to_reflex_id, _reflex_points, _vis_graph_reflex_reflex);
    double time_vis_planner = clock.TimeInSeconds();

    double time_total = time_reflex_vertices + time_vis_graph_reflex_reflex + time_vis_graph_reflex_reflex_to_bool + time_reflex_graph + time_vis_planner;

    _has_vis_graph_reflex = true;

    data::PTreeGeom rec;
    rec.Put("n_nodes", n_nodes);
    rec.Put("n_reflex", _n_reflex);
    rec.Put("n_convex", n_nodes - _n_reflex);
    rec.Put("time_reflex_vertices", time_reflex_vertices);
    rec.Put("time_vis_graph_reflex_reflex", time_vis_graph_reflex_reflex);
    rec.Put("time_vis_graph_reflex_reflex_to_bool", time_vis_graph_reflex_reflex_to_bool);
    rec.Put("time_reflex_graph", time_reflex_graph);
    rec.Put("time_vis_planner", time_vis_planner);
    rec.Put("time_total", time_total);
    return rec;
}

std::optional<data::PTreeGeom> PathFinder::PrecomputeReflexShortestPaths() {

    if (!_has_vis_graph_reflex) {
        return std::nullopt;
    }

    Clock clock;

    // Compute the shortest paths for the reflex graph.
    clock.Restart();
    ComputeShortestPathsDijkstraAllPairsReflex(_reflex_graph, _shortest_paths_lengths_reflex, _shortest_paths_predecessors_reflex);
    double time_total = clock.TimeInSeconds();

    _has_shortest_paths_reflex = true;

    data::PTreeGeom rec;
    rec.Put("time_total", time_total);
    return rec;
}

std::optional<data::PTreeGeom> PathFinder::ConstructCitiesVisibilityGraph(
    const trivis::Trivis &vis,
    trivis::geom::FPoints cities
) {

    _has_vis_graph_cities = false;
    _has_shortest_paths_cities = false;

    if (!_has_vis_graph_reflex) {
        return std::nullopt;
    }

    Clock clock;

    // Save cities.
    clock.Restart();
    _cities = std::move(cities);
    _n_cities = static_cast<int>(_cities.size());
    _cities_pl_results.resize(_n_cities);
    for (int city_id = 0; city_id < _n_cities; ++city_id) {
        _cities_pl_results[city_id] = vis.LocatePoint(_cities[city_id]);
    }
    double time_save_cities = clock.TimeInSeconds();

    // Compute city-reflex/reflex-city visibility graph.
    clock.Restart();
    _vis_graph_city_reflex = vis.PointVertexVisibilityGraph(_cities, _cities_pl_results, std::nullopt, _is_node_convex);
    for (auto &visible_reflex_vertices: _vis_graph_city_reflex) {
        for (int &visible_ver_id: visible_reflex_vertices) {
            visible_ver_id = _mesh_id_to_reflex_id[visible_ver_id];
        }
    }
    _vis_graph_reflex_city = std::vector<std::vector<int>>(_n_reflex);
    for (int i = 0; i < _n_cities; ++i) {
        for (int j = 0; j < _vis_graph_city_reflex[i].size(); ++j) {
            _vis_graph_reflex_city[_vis_graph_city_reflex[i][j]].push_back(i);
        }
    }
    double time_vis_graph_reflex_city = clock.TimeInSeconds();

    // Convert city-reflex/reflex-city visibility graph to bool matrix.
    clock.Restart();
    _vis_graph_city_reflex_bool_matrix = std::vector<std::vector<bool>>(_n_cities, std::vector<bool>(_n_reflex, false));
    _vis_graph_reflex_city_bool_matrix = std::vector<std::vector<bool>>(_n_reflex, std::vector<bool>(_n_cities, false));
    for (int city_id = 0; city_id < _n_cities; ++city_id) {
        for (int reflex_vertex_id: _vis_graph_city_reflex[city_id]) {
            _vis_graph_city_reflex_bool_matrix[city_id][reflex_vertex_id] = true;
            _vis_graph_reflex_city_bool_matrix[reflex_vertex_id][city_id] = true;
        }
    }
    double time_vis_graph_city_reflex_to_bool = clock.TimeInSeconds();

    // Compute city-city visibility graph.
    clock.Restart();
    _vis_graph_city_city = vis.PointPointVisibilityGraph(_cities, _cities_pl_results, std::nullopt);
    double time_vis_graph_city_city = clock.TimeInSeconds();

    // Convert city-city visibility graph to bool matrix.
    clock.Restart();
    _vis_graph_city_city_bool_matrix = std::vector<std::vector<bool>>(_n_cities, std::vector<bool>(_n_cities, false));
    for (int city_id_i = 0; city_id_i < _n_cities; ++city_id_i) {
        _vis_graph_city_city_bool_matrix[city_id_i][city_id_i] = true;
        for (int city_id_j: _vis_graph_city_city[city_id_i]) {
            _vis_graph_city_city_bool_matrix[city_id_i][city_id_j] = true;
            _vis_graph_city_city_bool_matrix[city_id_j][city_id_i] = true;
        }
    }
    double time_vis_graph_city_city_to_bool = clock.TimeInSeconds();

    double time_total = time_save_cities + time_vis_graph_reflex_city + time_vis_graph_city_reflex_to_bool + time_vis_graph_city_city + time_vis_graph_city_city_to_bool;

    _has_vis_graph_cities = true;

    data::PTreeGeom rec;
    rec.Put("n_cities", _n_cities);
    rec.Put("time_save_cities", time_save_cities);
    rec.Put("time_vis_graph_city_reflex", time_vis_graph_reflex_city);
    rec.Put("time_vis_graph_city_reflex_to_bool", time_vis_graph_city_reflex_to_bool);
    rec.Put("time_vis_graph_city_city", time_vis_graph_city_city);
    rec.Put("time_vis_graph_city_city_to_bool", time_vis_graph_city_city_to_bool);
    rec.Put("time_total", time_total);
    return rec;
}

std::optional<data::PTreeGeom> PathFinder::PrecomputeCitiesShortestPaths() {

    if (!_has_vis_graph_cities) {
        return std::nullopt;
    }

    Clock clock;
    ComputeShortestPathsDijkstraAllPairsCities(
        _reflex_graph,
        _reflex_points,
        _cities,
        _vis_graph_city_city,
        _vis_graph_city_reflex,
        _shortest_paths_lengths_cities,
        _shortest_paths_predecessors_cities);
    double time_total = clock.TimeInSeconds();

    _has_shortest_paths_cities = true;

    data::PTreeGeom rec;
    rec.Put("time_total", time_total);
    return rec;
}

double PathFinder::ShortestPathReflex(
    const trivis::Trivis &vis,
    int source_reflex_id,
    int target_reflex_id,
    trivis::geom::FPoints *point_path,
    std::vector<int> *id_path_no_endpoints
) {
    if (source_reflex_id == target_reflex_id) {
        if (point_path) {
            *point_path = {_reflex_points[source_reflex_id], _reflex_points[target_reflex_id]};
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::vector<int>{};
        }
        return 0.0;
    }
    if (_has_shortest_paths_reflex) {
        double length = _shortest_paths_lengths_reflex[source_reflex_id][target_reflex_id];
        std::vector<int> id_path_temp;
        if (point_path || id_path_no_endpoints) {
            id_path_temp = pathfinding::GetShortestIDPathAllPairsReflex(source_reflex_id, target_reflex_id, _shortest_paths_predecessors_reflex[source_reflex_id]);
        }
        if (point_path) {
            *point_path = utils::Select(_reflex_points, id_path_temp);
        }
        if (id_path_no_endpoints) {
            if (id_path_temp.size() <= 2) {
                *id_path_no_endpoints = std::vector<int>{};
            } else {
                *id_path_no_endpoints = std::vector<int>(id_path_temp.begin() + 1, id_path_temp.end() - 1);
            }
        }
        return length;
    }
    if (_has_vis_graph_reflex) {
        std::vector<int> id_path_temp;
        double length = _vis_planner.FindShortestPath(source_reflex_id, target_reflex_id, id_path_temp);
        if (point_path) {
            *point_path = utils::Select(_reflex_points, id_path_temp);
        }
        if (id_path_no_endpoints) {
            if (id_path_temp.size() <= 2) {
                *id_path_no_endpoints = std::vector<int>{};
            } else {
                *id_path_no_endpoints = std::vector<int>(id_path_temp.begin() + 1, id_path_temp.end() - 1);
            }
        }
        return length;
    }
    LOGF_ERR("[PathFinder::ShortestPathReflex] Could not get the shortest path!");
    return -1.0;
}

double PathFinder::ShortestPathCities(
    const trivis::Trivis &vis,
    int source_city_id,
    int target_city_id,
    trivis::geom::FPoints *point_path,
    std::vector<int> *id_path_no_endpoints
) {
    const auto &source_city = _cities[source_city_id];
    const auto &target_city = _cities[target_city_id];
    if (source_city_id == target_city_id) {
        if (point_path) {
            *point_path = {source_city, target_city};
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::vector<int>{};
        }
        return 0.0;
    }
    if (_has_shortest_paths_cities) {
        double length = pathfinding::GetShortestPathLengthAllPairsCities(source_city_id, target_city_id, _n_reflex, _shortest_paths_lengths_cities);
        std::vector<int> id_path_temp;
        if (id_path_no_endpoints || point_path) {
            id_path_temp = pathfinding::GetShortestIDPathAllPairsCities(source_city_id, target_city_id, _n_reflex, _shortest_paths_predecessors_cities);
        }
        if (point_path) {
            *point_path = pathfinding::ConvertIDPathToPointPathAllPairsCities(id_path_temp, _reflex_points, _cities);
        }
        if (id_path_no_endpoints) {
            if (id_path_temp.size() <= 2) {
                *id_path_no_endpoints = std::vector<int>{};
            } else {
                *id_path_no_endpoints = std::vector<int>(id_path_temp.begin() + 1, id_path_temp.end() - 1);
            }
        }
        return length;
    }
    if (_has_vis_graph_cities && _has_shortest_paths_reflex) {
        std::vector<int> id_path_no_endpoints_temp;
        double length = pathfinding::ComputeShortestPathDijkstraCities(
            source_city,
            target_city,
            _vis_graph_city_city_bool_matrix[source_city_id][target_city_id],
            _vis_graph_city_reflex[source_city_id],
            _vis_graph_city_reflex[target_city_id],
            _reflex_points,
            _shortest_paths_lengths_reflex,
            _shortest_paths_predecessors_reflex,
            id_path_no_endpoints_temp);
        if (point_path) {
            *point_path = pathfinding::ConvertIDPathToPointPathCities(id_path_no_endpoints_temp, _reflex_points, source_city, target_city);
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::move(id_path_no_endpoints_temp);
        }
        return length;
    }
    if (_has_vis_graph_cities && _has_vis_graph_reflex) {
        std::vector<int> id_path_no_cities_temp;
        _vis_planner.FindShortestPath(
            source_city,
            target_city,
            _vis_graph_city_city_bool_matrix[source_city_id][target_city_id],
            _vis_graph_city_reflex[source_city_id],
            _vis_graph_city_reflex[target_city_id],
            id_path_no_cities_temp);
        double length = _vis_planner.FindShortestPath(source_city, target_city, vis, id_path_no_cities_temp);
        if (point_path) {
            *point_path = pathfinding::ConvertIDPathToPointPathCities(id_path_no_cities_temp, _reflex_points, source_city, target_city);
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::move(id_path_no_cities_temp);
        }
        return length;
    }
    LOGF_ERR("[PathFinder::ShortestPathCities] Could not get the shortest path!");
    return -1.0;
}

double PathFinder::ShortestPathPoints(
    const trivis::Trivis &vis,
    const geom::FPoint &source_point,
    const geom::FPoint &target_point,
    trivis::geom::FPoints *point_path,
    std::vector<int> *id_path_no_endpoints
) {
    if (source_point == target_point) {
        if (point_path) {
            *point_path = {source_point, target_point};
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::vector<int>{};
        }
        return 0.0;
    }
    if (_has_shortest_paths_reflex) {

        auto source_point_pl = vis.LocatePoint(source_point);
        if (!source_point_pl.has_value()) {
            LOGF_ERR("[PathFinder::ShortestPathPoints] Source out of map!");
            return -1.0;
        }

        auto target_point_pl = vis.LocatePoint(target_point);
        if (!target_point_pl.has_value()) {
            LOGF_ERR("[PathFinder::ShortestPathPoints] Target out of map!");
            return -1.0;
        }

        std::vector<int> visible_vertices_source;
        { // Compute vertices visible from the source
            auto visible_vertices_source_temp = vis.VisibleVertices(source_point, source_point_pl.value(), std::nullopt, _is_node_convex);
            visible_vertices_source.reserve(visible_vertices_source_temp.size());
            for (int vertex_id: visible_vertices_source_temp) {
                visible_vertices_source.push_back(_mesh_id_to_reflex_id[vertex_id]);
            }
        }

        std::vector<int> visible_vertices_target;
        { // Compute vertices visible from the target
            auto visible_vertices_target_temp = vis.VisibleVertices(target_point, target_point_pl.value(), std::nullopt, _is_node_convex);
            visible_vertices_target.reserve(visible_vertices_target_temp.size());
            for (int vertex_id: visible_vertices_target_temp) {
                visible_vertices_target.push_back(_mesh_id_to_reflex_id[vertex_id]);
            }
        }

        bool visible = vis.IsVisible(source_point, source_point_pl.value(), target_point);

        std::vector<int> id_path_no_cities_temp;
        double length = pathfinding::ComputeShortestPathDijkstraCities(
            source_point,
            target_point,
            visible,
            visible_vertices_source,
            visible_vertices_target,
            _reflex_points,
            _shortest_paths_lengths_reflex,
            _shortest_paths_predecessors_reflex,
            id_path_no_cities_temp);
        if (point_path) {
            *point_path = pathfinding::ConvertIDPathToPointPathCities(id_path_no_cities_temp, _reflex_points, source_point, target_point);
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::move(id_path_no_cities_temp);
        }

        return length;
    }
    if (_has_vis_graph_reflex) {
        std::vector<int> id_path_no_cities_temp;
        double length = _vis_planner.FindShortestPath(source_point, target_point, vis, id_path_no_cities_temp);
        if (point_path) {
            *point_path = pathfinding::ConvertIDPathToPointPathCities(id_path_no_cities_temp, _reflex_points, source_point, target_point);
        }
        if (id_path_no_endpoints) {
            *id_path_no_endpoints = std::move(id_path_no_cities_temp);
        }
        return length;
    }
    LOGF_ERR("[PathFinder::ShortestPathPoints] Could not get the shortest path!");
    return -1.0;
}

void PathFinder::Clear() {
    // FILLED BY ConstructReflexVisibilityGraph
    _has_vis_graph_reflex = false;
    _n_reflex = 0;
    if (_is_node_convex.has_value()) {
        _is_node_convex.value().clear();
    }
    _reflex_points.clear();
    _reflex_id_to_mesh_id.clear();
    _mesh_id_to_reflex_id.clear();
    _vis_graph_reflex_reflex.clear();
    _vis_graph_reflex_reflex_bool_matrix.clear();
    _reflex_graph.clear();
    _vis_planner.Clear();
    // FILLED BY PrecomputeReflexShortestPaths
    _has_shortest_paths_reflex = false;
    _shortest_paths_lengths_reflex.clear();
    _shortest_paths_predecessors_reflex.clear();
    // FILLED BY ConstructReflexVisibilityGraph
    _has_vis_graph_cities = false;
    _n_cities = 0;
    _cities.clear();
    _vis_graph_city_reflex.clear();
    _vis_graph_city_reflex_bool_matrix.clear();
    _vis_graph_reflex_city.clear();
    _vis_graph_reflex_city_bool_matrix.clear();
    _vis_graph_city_city.clear();
    _vis_graph_city_city_bool_matrix.clear();
    // FILLED BY PrecomputeCitiesShortestPaths
    _has_shortest_paths_cities = false;
    _shortest_paths_lengths_cities.clear();
    _shortest_paths_predecessors_cities.clear();
}
