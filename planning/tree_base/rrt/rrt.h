/**
 * @file rrt_star.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief  RRT* algorithm implementation
 * @version 0.1
 * @date 2023-09-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_TREE_BASE_RRT_STAR_RRT_STAR_H_
#define PLANNING_TREE_BASE_RRT_STAR_RRT_STAR_H_

#include "planning/include/i_planning.h"
#include "planning/tree_base/include/common_tree_base.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

namespace planning
{
namespace tree_base
{

struct RRTCost
{
  RRTCost() : g(0), e(0), f(0) {}
  RRTCost(double g, double e) : g(g), e(e), f(g + e) {}

  double g; // cost from start node
  double e; // real distance from start node
  double f; // total cost
};

class RRT : public IPlanningWithLogging
{
public:
  RRT() {}
  RRT(const int max_iteration) : max_iteration_(max_iteration) {}
  ~RRT() {}
  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;
  Log GetLog() override;

private:
  Log log_;

  std::vector<std::shared_ptr<NodeParent<RRTCost>>> visited_nodes_;
  int max_iteration_{10000};
  int max_branch_length_{10};
  int min_branch_length_{5};
  int neighbor_radius_{15};
  int goal_radius_{5};
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_STAR_RRT_STAR_H_ */
