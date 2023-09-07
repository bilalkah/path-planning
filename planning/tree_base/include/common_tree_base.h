/**
 * @file common_tree_base.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLAN_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_
#define PLAN_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_

#include "planning/include/common_planning.h"

namespace planning
{
namespace tree_base
{

struct Cost
{
  Cost() : g(0), h(0), f(0) {}
  Cost(double g, double h) : g(g), h(h), f(g + h) {}

  double g; // cost from start node
  double h; // heuristic cost to goal node
  double f; // total cost
};

} // namespace tree_base
} // namespace planning

#endif /* PLAN_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_ */
