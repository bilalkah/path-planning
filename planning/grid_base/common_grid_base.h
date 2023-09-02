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

#ifndef PLANNING_GRID_BASE_COMMON_H_
#define PLANNING_GRID_BASE_COMMON_H_


#include "planning/common_planning.h"

namespace planning
{

namespace grid_base
{

/**
 * @brief Search space for path finding algorithms.
 *
 */
using SearchSpace = std::vector<std::pair<uint8_t, uint8_t>>;

/**
 * @brief Struct that generates both 4 and 8 directions search spaces.
 *
 */
struct SearchSpaceGenerator
{
  SearchSpace four_directions = {
      std::make_pair(0, 1),  // right
      std::make_pair(1, 0),  // down
      std::make_pair(0, -1), // left
      std::make_pair(-1, 0)  // up
  };

  SearchSpace eight_directions = {
      std::make_pair(0, 1),   // right
      std::make_pair(1, 0),   // down
      std::make_pair(0, -1),  // left
      std::make_pair(-1, 0),  // up
      std::make_pair(1, 1),   // down right
      std::make_pair(1, -1),  // down left
      std::make_pair(-1, -1), // up left
      std::make_pair(-1, 1)   // up right
  };
};

} // namespace grid_base
} // namespace planning

#endif /* PLANNING_GRID_BASE_COMMON_H_ */