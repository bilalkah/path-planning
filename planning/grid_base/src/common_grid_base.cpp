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

SearchSpace GetFourDirection() { return {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}; }

SearchSpace GetEightDirection()
{
  return {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
}

} // namespace grid_base
} // namespace planning
