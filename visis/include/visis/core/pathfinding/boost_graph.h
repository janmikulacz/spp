/**
 * File:   boost_graph.h
 *
 * Date:   21.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_PATHFINDING_BOOST_GRAPH_H_
#define VISIS_CORE_PATHFINDING_BOOST_GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "visis/core/guards/reflex_vertex.h"
#include "trivis/trivis.h"

namespace visis::core::pathfinding {

using BoostGraphOutEdgeListType = boost::vecS;
using BoostGraphVertexListType = boost::vecS;
using BoostGraphDirectedProperty = boost::undirectedS;
using BoostGraphVertexProperty = boost::no_property;
using BoostGraphEdgeProperty = boost::property<boost::edge_weight_t, double>;
using BoostGraphProperty = boost::no_property;
using BoostGraphEdgeListType = boost::listS;
using BoostGraph = boost::adjacency_list<
    BoostGraphOutEdgeListType,
    BoostGraphVertexListType,
    BoostGraphDirectedProperty,
    BoostGraphVertexProperty,
    BoostGraphEdgeProperty,
    BoostGraphProperty,
    BoostGraphEdgeListType
>;
using BoostGraphEdgeDescriptor = boost::graph_traits<BoostGraph>::edge_descriptor;

BoostGraph ConstructReflexGraph(
    const std::vector<std::vector<int>> &vis_graph_reflex_reflex,
    const trivis::geom::FPoints &reflex_vertices_points
);

}

#endif //VISIS_CORE_PATHFINDING_BOOST_GRAPH_H_
