/**
 * File:   load_mesh.h
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_DATA_LOADING_LOAD_MESH_H_
#define VISIS_DATA_LOADING_LOAD_MESH_H_

#include <string>

#include "trivis/trivis.h"

namespace visis::data {

bool LoadTriMesh(
    const std::string &file,
    const trivis::geom::FPoints &points,
    trivis::mesh::TriMesh &mesh
) noexcept(false);

std::string LoadTriMeshSafely(
    const std::string &file,
    const trivis::geom::FPoints &points,
    trivis::mesh::TriMesh &mesh
);

}

#endif //VISIS_DATA_LOADING_LOAD_MESH_H_
