/**
 * File:   load_map.h
 *
 * Date:   29.11.2021
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_DATA_LOADING_LOAD_MAP_H_
#define VISIS_DATA_LOADING_LOAD_MAP_H_

#include <string>
#include <optional>

#include "trivis/trivis.h"

namespace visis::data {

/**
 *
 * Loads map from file in the following format:
 *
 *      [SCALE]
 *      <default_scale>
 *
 *      [BORDER]
 *      <x1> <y1>
 *      <x2> <y2>
 *      ...
 *      <xn> <yn>
 *
 *      [OBSTACLE]
 *      <x1> <y1>
 *      <x2> <y2>
 *      ...
 *      <xi> <yi>
 *
 *      ...
 *
 *      [OBSTACLE]
 *      <x1> <y1>
 *      <x2> <y2>
 *      ...
 *      <xj> <yj>
 *
 * @param file
 * @param border
 * @param holes
 * @param scale
 * @return
 */
bool LoadPolyMap(
    const std::string &file,
    trivis::geom::PolyMap &poly_map,
    std::optional<double> scale = std::nullopt
) noexcept(false);

std::optional<trivis::geom::PolyMap> LoadPolyMapSafely(
    const std::string &file,
    std::optional<double> scale = std::nullopt
);

}

#endif //VISIS_DATA_LOADING_LOAD_MAP_H_
