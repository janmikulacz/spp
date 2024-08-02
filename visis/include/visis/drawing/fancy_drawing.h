/**
 * File:   fancy_drawing.h
 *
 * Date:   08.11.2022
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_DRAWING_FANCY_DRAWING_H_
#define VISIS_DRAWING_FANCY_DRAWING_H_

#include "visis/drawing/drawing.h"

#include "trivis/trivis.h"

namespace visis::drawing {

void FancyDrawMesh(
    const MapDrawer &drawer,
    const trivis::Trivis &vis
);

struct VisibilityRegionColors {
    RGB point = kColorDeepSkyBlue;
    RGB base = kColorLightSkyBlue;
    RGB edge_obstacle = kColorRed;
    RGB edge_free = kColorLimeGreen;
    RGB edge_arc = kColorBlueViolet;
    RGB edge_other = kColorDeepPink;
    RGB vertex_node = kColorOrange;
    RGB vertex_intersection = kColorYellow;
};

void FancyDrawRadialVisibilityRegion(
    const MapDrawer &drawer,
    const trivis::RadialVisibilityRegion &vis_reg,
    const VisibilityRegionColors &colors = VisibilityRegionColors{},
    double w = 1.0,
    double s = 1.0
);

void FancyDrawRadialVisibilityRegion(
    const MapDrawer &drawer,
    const trivis::RadialVisibilityRegion &vis_reg,
    const RGB &c,
    double w = 1.0,
    double s = 1.0
);

void FancyDrawPath(
    const MapDrawer &drawer,
    const trivis::geom::FPoints &path,
    int n_reflex_vertices,
    const std::vector<int> &ids,
    double w = 1.0,
    double s = 1.0
);

}

#endif //VISIS_DRAWING_FANCY_DRAWING_H_
