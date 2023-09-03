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
#include <cstddef>

namespace planning
{

namespace grid_base
{

/**
 * @brief Search space for path finding algorithms.
 *
 */
using SearchSpace = std::vector<std::pair<int8_t, int8_t>>;

/**
 * @brief Struct that generates both 4 and 8 directions search spaces.
 *
 */
struct SearchSpaceGenerator
{
  SearchSpaceGenerator();

  SearchSpace four_directions;
  SearchSpace eight_directions;
};

} // namespace grid_base
} // namespace planning

#endif /* PLANNING_GRID_BASE_INCLUDE_COMMON_GRID_BASE_H_ */