/**
 * File:    random_colors.h
 *
 * Date:    07.04.2021
 * Author:  Jan Mikula
 * E-mail:  jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_DRAWING_RANDOM_COLORS_H_
#define VISIS_DRAWING_RANDOM_COLORS_H_

#include "visis/drawing/colors.h"

#include <vector>

namespace visis::drawing {

std::vector<RGB> RandomColors(int n, int seed = 42);

}

#endif //VISIS_DRAWING_RANDOM_COLORS_H_
