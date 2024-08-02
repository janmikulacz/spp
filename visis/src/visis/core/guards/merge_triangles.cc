/**
 * File:   merge_triangles.cc
 *
 * Date:   14.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/merge_triangles.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

using namespace trivis;

ConvexPartition MakeConvexPartition(
    int triangle_id,
    const mesh::TriMesh &mesh,
    std::vector<bool> &tabu_triangles,
    std::vector<bool> &tabu_tri_edges
) {
    ConvexPartition partition;
    const auto &tri_edges = mesh.edges;
    const auto &triangles = mesh.triangles;
    for (int tri_edge_id: triangles[triangle_id].edges) {
        const auto &tri_edge = tri_edges[tri_edge_id];
        ConvexPartitionEdge par_edge;
        par_edge.closed = tri_edge.is_boundary() || tabu_tri_edges[tri_edge_id];
        par_edge.id = tri_edge_id;
        if (tri_edge.triangles[0] != triangle_id) {
            std::swap(par_edge.e_tri_0, par_edge.e_tri_1);
        }
        const auto &p0 = mesh.point(tri_edge.opposites[par_edge.e_tri_0]);
        const auto &p1 = mesh.point(tri_edge.vertices[par_edge.e_v_r]);
        const auto &p2 = mesh.point(tri_edge.vertices[par_edge.e_v_l]);
        if (!TurnsLeft(p0, p1, p2)) {
            std::swap(par_edge.e_v_l, par_edge.e_v_r);
        }
        partition.push_back(par_edge);
        tabu_tri_edges[tri_edge_id] = true;
    }
    tabu_triangles[triangle_id] = true;
    return partition;
}

bool IsExpansionConvex(
    int i,
    const ConvexPartition &partition,
    const mesh::TriMesh &mesh
) {
    const auto &tri_edges = mesh.edges;
    const auto &par_edge = partition[i];
    int tri_edge_id = par_edge.id;
    const auto &tri_edge = tri_edges[tri_edge_id];
    int n_par_edges = static_cast<int>(partition.size());
    int i_prev = (i == 0) ? n_par_edges - 1 : i - 1;
    int i_next = (i == n_par_edges - 1) ? 0 : i + 1;
    const auto &par_edge_prev = partition[i_prev];
    const auto &par_edge_next = partition[i_next];
    int tri_edge_prev_id = par_edge_prev.id;
    int tri_edge_next_id = par_edge_next.id;
    const auto &tri_edge_prev = tri_edges[tri_edge_prev_id];
    const auto &tri_edge_next = tri_edges[tri_edge_next_id];
    int v0 = tri_edge_prev.vertices[par_edge_prev.e_v_r];
    int v1 = tri_edge_prev.vertices[par_edge_prev.e_v_l];
    int v2 = tri_edge.opposites[par_edge.e_tri_1];
    int v3 = tri_edge_next.vertices[par_edge_next.e_v_r];
    int v4 = tri_edge_next.vertices[par_edge_next.e_v_l];
    const auto &p0 = mesh.point(v0);
    const auto &p1 = mesh.point(v1);
    const auto &p2 = mesh.point(v2);
    const auto &p3 = mesh.point(v3);
    const auto &p4 = mesh.point(v4);
    return !geom::TurnsRight(p0, p1, p2) && !geom::TurnsRight(p2, p3, p4);
}

void ExpandEdge(
    int i,
    const mesh::TriMesh &mesh,
    ConvexPartition &partition,
    std::vector<bool> &tabu_triangles,
    std::vector<bool> &tabu_tri_edges
) {
    const auto &tri_edges = mesh.edges;
    const auto &triangles = mesh.triangles;
    const auto &par_edge = partition[i];
    int tri_edge_id = par_edge.id;
    const auto &tri_edge = tri_edges[tri_edge_id];
    int n_par_edges = static_cast<int>(partition.size());
    int i_prev = (i == 0) ? n_par_edges - 1 : i - 1;
    int i_next = (i == n_par_edges - 1) ? 0 : i + 1;
    int tri_v_l = tri_edge.vertices[par_edge.e_v_l];
    int tri_v_r = tri_edge.vertices[par_edge.e_v_r];
    int triangle_id = tri_edge.triangles[par_edge.e_tri_1];
    const auto &triangle = triangles[triangle_id];
    ConvexPartitionEdge new_par_edge_0, new_par_edge_1;
    for (int k = 0; k < 3; ++k) {
        int id = triangle.edges[k];
        if (id == tri_edge_id) {
            new_par_edge_0.id = triangle.edges[(k + 1) % 3];
            new_par_edge_1.id = triangle.edges[(k + 2) % 3];
            break;
        }
    }
    const auto &new_edge0 = tri_edges[new_par_edge_0.id];
    const auto &new_edge1 = tri_edges[new_par_edge_1.id];
    new_par_edge_0.closed = new_edge0.is_boundary() || tabu_tri_edges[new_par_edge_0.id];
    new_par_edge_1.closed = new_edge1.is_boundary() || tabu_tri_edges[new_par_edge_1.id];
    if (new_edge0.triangles[0] != triangle_id) {
        std::swap(new_par_edge_0.e_tri_0, new_par_edge_0.e_tri_1);
    }
    if (new_edge1.triangles[0] != triangle_id) {
        std::swap(new_par_edge_1.e_tri_0, new_par_edge_1.e_tri_1);
    }
    if (new_edge0.vertices[0] == tri_v_r) {
        std::swap(new_par_edge_0.e_v_l, new_par_edge_0.e_v_r);
    }
    if (new_edge1.vertices[0] != tri_v_l) {
        std::swap(new_par_edge_1.e_v_l, new_par_edge_1.e_v_r);
    }
    partition[i_prev].convex_valid = false;
    partition[i_next].convex_valid = false;
    partition[i] = new_par_edge_1;
    partition.insert(partition.begin() + i, new_par_edge_0);
    tabu_tri_edges[new_par_edge_0.id] = true;
    tabu_tri_edges[new_par_edge_1.id] = true;
    tabu_triangles[triangle_id] = true;
}

ConvexPartitions guards::MergeTrianglesToConvexPartitions(
    const mesh::TriMesh &mesh,
    const std::vector<int> &triangle_ids,
    const std::vector<double> &values,
    MergeTrianglesSelectionRule selection_rule
) {
    const auto &tri_edges = mesh.edges;
    const auto &triangles = mesh.triangles;
    int n_triangles = static_cast<int>(triangles.size());
    int n_tri_edges = static_cast<int>(tri_edges.size());
    std::vector<bool> tabu_triangles(n_triangles, false); // triangles included in some partition
    std::vector<bool> tabu_tri_edges(n_tri_edges, false); // triangle edges included in some partition
    ConvexPartitions partitions;
    for (int init_triangle_id: triangle_ids) {
        if (tabu_triangles[init_triangle_id]) {
            continue;
        }
        auto partition = MakeConvexPartition(init_triangle_id, mesh, tabu_triangles, tabu_tri_edges);
        while (true) {
            int par_expansion_edge = -1;
            int n_par_edges = static_cast<int>(partition.size());
            double min_value = std::numeric_limits<double>::max();
            for (int i = 0; i < n_par_edges; ++i) {
                auto &par_edge = partition[i];
                if (par_edge.closed) {
                    continue;
                }
                int edge_id = par_edge.id;
                const auto &edge = tri_edges[edge_id];
                if (!par_edge.convex_valid) {
                    par_edge.convex = IsExpansionConvex(i, partition, mesh);
                    par_edge.convex_valid = true;
                    if (!par_edge.convex) {
                        par_edge.closed = true;
                        continue;
                    }
                }
                double value = values[edge.triangles[par_edge.e_tri_1]];
                if (value < min_value) {
                    min_value = value;
                    par_expansion_edge = i;
                    if (selection_rule == MergeTrianglesSelectionRule::kFirst) {
                        break;
                    }
                }
            }
            if (par_expansion_edge == -1) {
                break;
            }
            ExpandEdge(par_expansion_edge, mesh, partition, tabu_triangles, tabu_tri_edges);
        }
        partitions.push_back(std::move(partition));
    }
    return partitions;
}

ConvexPartitions guards::MergeTrianglesToConvexPartitions(
    const trivis::Trivis &vis,
    MergeTrianglesOrdering ordering,
    MergeTrianglesSelectionRule selection_rule
) {
    const auto &mesh = vis.mesh();
    const auto &tri_edges = mesh.edges;
    const auto &triangles = mesh.triangles;
    const auto &tri_polygons = vis.triangles();
    int n_triangles = static_cast<int>(triangles.size());
    std::vector<double> values(n_triangles, 0.0);
    auto triangle_ids = utils::Range(n_triangles);
    if (ordering != MergeTrianglesOrdering::kDefault) {

        if (ordering == MergeTrianglesOrdering::kEars
            || ordering == MergeTrianglesOrdering::kEarsSmall
            || ordering == MergeTrianglesOrdering::kEarsLarge
            || ordering == MergeTrianglesOrdering::kOpen
            || ordering == MergeTrianglesOrdering::kOpenSmall
            || ordering == MergeTrianglesOrdering::kOpenLarge) {

            std::vector<int> n_obstacles(n_triangles, 0);

            for (int i = 0; i < n_triangles; ++i) {
                for (int edge_id: triangles[i].edges) {
                    if (tri_edges[edge_id].is_boundary()) {
                        ++n_obstacles[i];
                    }
                }
            }

            if (ordering == MergeTrianglesOrdering::kEars
                || ordering == MergeTrianglesOrdering::kEarsSmall
                || ordering == MergeTrianglesOrdering::kEarsLarge) {

                for (int i = 0; i < n_triangles; ++i) {
                    values[i] += static_cast<double>(3 - n_obstacles[i]);
                }
            }

            if (ordering == MergeTrianglesOrdering::kOpen
                || ordering == MergeTrianglesOrdering::kOpenSmall
                || ordering == MergeTrianglesOrdering::kOpenLarge) {

                for (int i = 0; i < n_triangles; ++i) {
                    values[i] += static_cast<double>(n_obstacles[i]);
                }
            }

        }

        if (ordering == MergeTrianglesOrdering::kSmall
            || ordering == MergeTrianglesOrdering::kEarsSmall
            || ordering == MergeTrianglesOrdering::kOpenSmall
            || ordering == MergeTrianglesOrdering::kLarge
            || ordering == MergeTrianglesOrdering::kEarsLarge
            || ordering == MergeTrianglesOrdering::kOpenLarge) {

            std::vector<double> areas(n_triangles, 0.0);

            double max_area = 0.0;
            for (int i = 0; i < n_triangles; ++i) {
                areas[i] = geom::Area(tri_polygons[i]);
                if (areas[i] > max_area) {
                    max_area = areas[i];
                }
            }

            if (ordering == MergeTrianglesOrdering::kSmall
                || ordering == MergeTrianglesOrdering::kEarsSmall
                || ordering == MergeTrianglesOrdering::kOpenSmall) {

                for (int i = 0; i < n_triangles; ++i) {
                    values[i] += 0.01 + 0.98 * areas[i] / max_area;
                }
            }

            if (ordering == MergeTrianglesOrdering::kLarge
                || ordering == MergeTrianglesOrdering::kEarsLarge
                || ordering == MergeTrianglesOrdering::kOpenLarge) {

                for (int i = 0; i < n_triangles; ++i) {
                    values[i] += 0.99 - 0.98 * areas[i] / max_area;
                }
            }
        }

        std::sort(triangle_ids.begin(), triangle_ids.end(), [values](int tri_id_0, int tri_id_1) {
            return values[tri_id_0] < values[tri_id_1];
        });
    }
    return MergeTrianglesToConvexPartitions(vis.mesh(), triangle_ids, values, selection_rule);
}

trivis::geom::FPolygons guards::ToPolygons(
    const ConvexPartitions &partitions,
    const mesh::TriMesh &mesh
) {
    const auto &tri_edges = mesh.edges;
    int n_partitions = static_cast<int>(partitions.size());
    geom::FPolygons polygons(n_partitions);
    for (int i = 0; i < n_partitions; ++i) {
        const auto &partition = partitions[i];
        int partition_size = static_cast<int>(partition.size());
        auto &polygon = polygons[i];
        polygon.resize(partition_size);
        for (int j = 0; j < partition_size; ++j) {
            const auto &p_edge = partition[j];
            polygon[j] = mesh.point(tri_edges[p_edge.id].vertices[p_edge.e_v_l]);
        }
    }
    return polygons;
}
