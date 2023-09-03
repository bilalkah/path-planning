/**
 * @file common.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Common definitions for grid base planning module.
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_GRID_BASE_INCLUDE_COMMON_GRID_BASE_H_
#define PLANNING_GRID_BASE_INCLUDE_COMMON_GRID_BASE_H_

#include "planning/include/common_planning.h"
#include <array>
#include <cstddef>

namespace planning
{

namespace grid_base
{

using Direction = std::array<int8_t, 2>;
using Directions4 = std::array<Direction, 4>;
using Directions8 = std::array<Direction, 8>;

constexpr Directions4 four_directions{{
    Direction{0, 1},  // right
    Direction{1, 0},  // down
    Direction{0, -1}, // left
    Direction{-1, 0}  // up
}};

// eight directions
constexpr Directions8 eight_directions{{
    Direction{0, 1},   // right
    Direction{1, 0},   // down
    Direction{0, -1},  // left
    Direction{-1, 0},  // up
    Direction{1, 1},   // down right
    Direction{1, -1},  // down left
    Direction{-1, -1}, // up left
    Direction{-1, 1}   // up right
}};

} // namespace grid_base
} // namespace planning

#endif /* PLANNING_GRID_BASE_INCLUDE_COMMON_GRID_BASE_H_ */