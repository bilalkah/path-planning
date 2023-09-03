/**
 * @file common_grid_base.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/grid_base/include/common_grid_base.h"

namespace planning
{
namespace grid_base
{

SearchSpaceGenerator::SearchSpaceGenerator()
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
}

} // namespace grid_base
} // namespace planning