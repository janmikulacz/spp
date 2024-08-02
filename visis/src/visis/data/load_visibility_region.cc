/**
 * File:   load_visibility_region.cc
 *
 * Date:   27.04.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/data/load_visibility_region.h"

#include <fstream>
#include <iostream>

using namespace visis;
using namespace visis::data;

using namespace trivis;
using namespace trivis::geom;

bool data::LoadVisibilityRegion(
    const std::string &file,
    RadialVisibilityRegion &visibility_region
) noexcept(false) {
    std::ifstream ifs(file.c_str());
    if (ifs.fail()) {
        return false;
    }
    std::string token;
    while (true) {
        ifs >> token;
        if (ifs.eof()) {
            break;
        }
        if (token == "[SEED]") {
            ifs >> visibility_region.seed.x;
            ifs >> visibility_region.seed.y;
            int seed_id = visibility_region.seed_id.has_value() ? visibility_region.seed_id.value() : -1;
            ifs >> seed_id;
            continue;
        }
        if (token == "[RADIUS]") {
            double vis_radius = visibility_region.radius.has_value() ? visibility_region.radius.value() : -1.0;
            ifs >> vis_radius;
            continue;
        }
        if (token == "[VERTICES]") {
            int n_vertices;
            ifs >> n_vertices;
            visibility_region.vertices.resize(n_vertices);
            for (int i = 0; i < n_vertices; ++i) {
                int vertex_id;
                ifs >> vertex_id;
                auto &v = visibility_region.vertices[vertex_id];
                ifs >> v.point.x;
                ifs >> v.point.y;
                ifs >> v.edge_flag;
                ifs >> v.vertex_flag;
            }
            continue;
        }
    }
    return true;
}

std::string data::LoadVisibilityRegionSafely(
    const std::string &file,
    RadialVisibilityRegion &visibility_region
) {
    try {
        if (!LoadVisibilityRegion(file, visibility_region)) {
            return "File " + file + " could not be opened or found.";
        }
    } catch (const std::invalid_argument &e) {
        return "Error while loading " + file + ":\n" + e.what();
    }
    return "ok";
}
