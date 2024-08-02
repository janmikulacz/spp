/**
 * File:   reflex_vertex.h
 *
 * Date:   24.03.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_GUARDS_REFLEX_VERTEX_H_
#define VISIS_CORE_GUARDS_REFLEX_VERTEX_H_

#include "trivis/trivis.h"

namespace visis::core::guards {

trivis::geom::FPoint MovePointAwayFromVertex(
    const trivis::Trivis &vis,
    int vertex_id_prev,
    int vertex_id,
    int vertex_id_next,
    double dist_from_vertex
);

}

#endif //VISIS_CORE_GUARDS_REFLEX_VERTEX_H_
