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
#include <vector>

namespace planning
{

namespace grid_base
{

using SearchSpace = std::vector<std::array<int8_t, 2>>;

SearchSpace GetFourDirection();

SearchSpace GetEightDirection();

} // namespace grid_base
} // namespace planning

#endif /* PLANNING_GRID_BASE_INCLUDE_COMMON_GRID_BASE_H_ */
