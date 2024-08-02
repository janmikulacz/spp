/**
 * File:   merge_triangles.h
 *
 * Date:   14.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_MERGE_TRIANGLES_H_
#define VISIS_CORE_GUARDS_MERGE_TRIANGLES_H_

#include "trivis/trivis.h"

namespace visis::core::guards {

enum class MergeTrianglesSelectionRule : int {
    kBest,
    kFirst
};

enum class MergeTrianglesOrdering : int {
    kDefault,
    kSmall,
    kLarge,
    kEars,
    kEarsSmall,
    kEarsLarge,
    kOpen,
    kOpenSmall,
    kOpenLarge
};

struct ConvexPartitionEdge {
    bool closed = false; // if true then the edge will never be expanded
    bool convex = false; // if true then the edge expansion is convex
    bool convex_valid = false; // if true then the above flag is valid, otherwise it must be resolved
    int id = -1; // mesh edge id
    int e_tri_0 = 0; // origin triangle id (in mesh edge)
    int e_tri_1 = 1; // expansion triangle id (in mesh edge)
    int e_v_l = 0; // left node id (in mesh edge)
    int e_v_r = 1; // right node id (in mesh edge)
};

using ConvexPartition = std::vector<ConvexPartitionEdge>;
using ConvexPartitions = std::vector<ConvexPartition>;

ConvexPartitions MergeTrianglesToConvexPartitions(
    const trivis::mesh::TriMesh &mesh,
    const std::vector<int> &triangle_ids,
    const std::vector<double> &values,
    MergeTrianglesSelectionRule selection_rule = MergeTrianglesSelectionRule::kFirst
);

ConvexPartitions MergeTrianglesToConvexPartitions(
    const trivis::Trivis &vis,
    MergeTrianglesOrdering ordering = MergeTrianglesOrdering::kDefault,
    MergeTrianglesSelectionRule selection_rule = MergeTrianglesSelectionRule::kFirst
);

trivis::geom::FPolygons ToPolygons(
    const ConvexPartitions &partitions,
    const trivis::mesh::TriMesh &mesh
);

}

#endif //VISIS_CORE_GUARDS_MERGE_TRIANGLES_H_
