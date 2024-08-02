/**
 * File:   load_map.cc
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/data/load_map.h"

#include <fstream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include "visis/log/log.h"

using namespace visis;
using namespace visis::data;

using namespace trivis;
using namespace trivis::geom;

namespace ba = boost::algorithm;

bool LoadMapNewFormat(
    std::ifstream &ifs,
    PolyMap &poly_map
) noexcept(false) {
    std::string token;
    FPoints points;
    FPolygon border;
    FPolygons holes;
    while (true) {
        ifs >> token;
        if (ifs.eof()) {
            break;
        }
        if (ba::to_upper_copy(token) == "[NAME]") {
            // can be ignored
            continue;
        }
        if (ba::to_upper_copy(token) == "[LIMITS]") {
            // can be ignored
            continue;
        }
        if (ba::to_upper_copy(token) == "[POINTS]") {
            int n_points;
            ifs >> n_points;
            points.reserve(n_points);
            int id;
            double x, y;
            for (int i = 0; i < n_points; ++i) {
                ifs >> id;
                ifs >> x;
                ifs >> y;
                points.emplace_back(x, y);
            }
            continue;
        }
        if (ba::to_upper_copy(token) == "[BORDER]") {
            int n_border;
            ifs >> n_border;
            border.reserve(n_border);
            int id;
            int point_id;
            for (int i = 0; i < n_border; ++i) {
                ifs >> id;
                ifs >> point_id;
                border.push_back(points[point_id]);
            }
            continue;
        }
        if (ba::to_upper_copy(token) == "[NUM_HOLES]") {
            int n_holes;
            ifs >> n_holes;
            holes.resize(n_holes);
            continue;
        }
        if (ba::to_upper_copy(token) == "[HOLE]") {
            int hole_id, n_hole;
            ifs >> hole_id;
            ifs >> n_hole;
            auto &hole = holes[hole_id];
            hole.reserve(n_hole);
            int id;
            int point_id;
            for (int i = 0; i < n_hole; ++i) {
                ifs >> id;
                ifs >> point_id;
                hole.push_back(points[point_id]);
            }
            continue;
        }
    }
    // Save to output.
    poly_map = PolyMap(border, holes, 1.0);
    return true;
}

bool data::LoadPolyMap(
    const std::string &file,
    PolyMap &poly_map,
    std::optional<double> scale
) noexcept(false) {
    std::ifstream ifs(file.c_str());
    if (ifs.fail()) {
        return false;
    }
    std::string token;
    FPolygon curr_polygon;
    bool processing_border = false;
    bool processing_obstacle = false;
    bool processing_map_points = false;
    bool format_map_points = false;
    Points<double> points;
    FPolygon border;
    FPolygons holes;
    double d_max = std::numeric_limits<double>::max();
    double x_min = -d_max, x_max = d_max, y_min = -d_max, y_max = d_max;
    while (true) {
        ifs >> token;
        if (ba::to_upper_copy(token) == "[NAME]") {
            return LoadMapNewFormat(ifs, poly_map);
        }
        if (ba::to_upper_copy(token) == "[INFO]") {
            continue;
        }
        if (ba::to_upper_copy(token) == "FORMAT=MAP_POINTS") {
            format_map_points = true;
            if (!scale || scale < 0.0) {
                scale = 1.0;
            }
            continue;
        }
        if (!format_map_points) { // FORMAT: SIMPLE
            // Save the border flag.
            if (ba::to_upper_copy(token) == "[BORDER]") {
                processing_border = true;
            }
            // If the next border/obstacle polygon or the end of the file is detected, ...
            // ... then save the current polygon to the map if it is non-empty.
            if (ba::to_upper_copy(token) == "[BORDER]" || ba::to_upper_copy(token) == "[OBSTACLE]" || ifs.eof()) {
                if (!curr_polygon.empty()) {
                    // Check polygon orientations.
                    if (processing_border) {
                        // Border should return true.
                        if (!OrientationCounterClockwise(curr_polygon)) {
                            // Fix the orientation if wrong.
                            ChangeOrientation(curr_polygon);
                        }
                        processing_border = false;
                        border = curr_polygon;
                        ComputeLimits(border, x_min, x_max, y_min, y_max);
                    } else {
                        // Obstacle should return false.
                        if (OrientationCounterClockwise(curr_polygon)) {
                            // Fix the orientation if wrong.
                            ChangeOrientation(curr_polygon);
                        }
                        holes.push_back(curr_polygon);
                    }
                    // Add the polygon to the map.
                    curr_polygon.clear();
                    if (ifs.eof()) break;
                }
            } else if (ba::to_upper_copy(token) == "[SCALE]") {
                ifs >> token;
                if (!scale || scale < 0.0) { // Load scale only if it is not given.
                    scale = std::stod(token);
                }
            } else if (!ifs.eof()) { // Assuming line with two double coordinates.
                if (!scale) scale = 1.0;
                double x, y;
                x = std::stod(token) * *scale;
                ifs >> token;
                y = std::stod(token) * *scale;
                curr_polygon.emplace_back(std::min(std::max(x_min, x), x_max), std::min(std::max(y_min, y), y_max));
            }
        } else { // FORMAT: MAP POINTS

            if (ba::to_upper_copy(token) == "[MAP_BORDER]" || ba::to_upper_copy(token) == "[MAP_OBSTACLE]" || ifs.eof()) {
                if (!curr_polygon.empty()) {
                    // Check polygon orientations.
                    if (processing_border) {
                        // Border should return true.
                        if (!OrientationCounterClockwise(curr_polygon)) {
                            // Fix the orientation if wrong.
                            ChangeOrientation(curr_polygon);
                        }
                        processing_border = false;
                        border = curr_polygon;
                        ComputeLimits(border, x_min, x_max, y_min, y_max);
                    } else {
                        // Obstacle should return false.
                        if (OrientationCounterClockwise(curr_polygon)) {
                            // Fix the orientation if wrong.
                            ChangeOrientation(curr_polygon);
                        }
                        holes.push_back(curr_polygon);
                    }
                    curr_polygon.clear();
                }
                if (ifs.eof()) {
                    break;
                }
            }

            if (ba::to_upper_copy(token) == "[MAP_POINTS]") {
                processing_map_points = true;
                processing_border = false;
                processing_obstacle = false;
            } else if (ba::to_upper_copy(token) == "[MAP_BORDER]") {
                processing_map_points = false;
                processing_border = true;
                processing_obstacle = false;
            } else if (ba::to_upper_copy(token) == "[MAP_OBSTACLE]") {
                processing_map_points = false;
                processing_border = false;
                processing_obstacle = true;
            } else if (ba::to_upper_copy(token) == "[MAP_CONVEX_REGION]" ||
                ba::to_upper_copy(token) == "[MESH_NODES]" ||
                ba::to_upper_copy(token) == "[MESH_TRIANGLES]" ||
                ba::to_upper_copy(token) == "[MESH_BOUNDARY_NODES]" ||
                ba::to_upper_copy(token) == "[TRIMESH_CONVEX_REGION]" ||
                ba::to_upper_copy(token) == "[TRIMESH_CONVEX_REGION_TRIANGLES]") {
                processing_map_points = false;
                processing_border = false;
                processing_obstacle = false;
            } else if (processing_map_points) {
                if (!scale) scale = 1.0;
                double x, y;
                ifs >> x;
                ifs >> y;
                points.emplace_back(std::min(std::max(x_min, x * *scale), x_max), std::min(std::max(y_min, y * *scale), y_max));
            } else if (processing_border || processing_obstacle) {
                int idx = std::stoi(token);
                curr_polygon.push_back(points[idx]);
            }
        }
    }
    poly_map = PolyMap(border, holes, *scale);
    return true;
}

std::optional<PolyMap> data::LoadPolyMapSafely(
    const std::string &file,
    std::optional<double> scale
) {
    PolyMap map;
    try {
        if (!LoadPolyMap(file, map, scale)) {
            LOGF_ERR("File " << file << " could not be opened or found.");
            return std::nullopt;
        }
    } catch (const std::invalid_argument &e) {
        LOGF_ERR("Error while loading " << file << ":\n" << e.what());
        return std::nullopt;
    }
    return map;
}
