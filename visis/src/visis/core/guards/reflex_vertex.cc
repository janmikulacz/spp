/**
 * File:   reflex_node.cc
 *
 * Date:   24.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/reflex_vertex.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

using namespace trivis;

geom::FPoint guards::MovePointAwayFromVertex(
    const trivis::Trivis &vis,
    int vertex_id_prev,
    int vertex_id,
    int vertex_id_next,
    double dist_from_vertex
) {
    const auto &mesh = vis.mesh();
    const auto &p_prev = mesh.point(vertex_id_prev);
    const auto &p = mesh.point(vertex_id);
    const auto &p_next = mesh.point(vertex_id_next);
    auto direction = -(((p_prev - p).CopyNormalized() + (p_next - p).CopyNormalized()) / 2.0).CopyNormalized();
    auto intersection = vis.ShootRay(vertex_id, direction);
    if (intersection.has_value() && (intersection->edge_id || intersection->ver_id)) {
        double half_dist_to_obstacle = p.DistanceTo(intersection->p) / 2.0;
        if (dist_from_vertex >= half_dist_to_obstacle) {
            return (p + intersection->p) / 2.0;
        }
    }
    return p + (direction * dist_from_vertex);
}
