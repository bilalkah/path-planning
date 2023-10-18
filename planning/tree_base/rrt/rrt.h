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

class RRT : public IPlanningWithLogging
{
public:
  RRT() {}
  RRT(const int max_iteration) : max_iteration_(max_iteration) {}
  RRT(const int max_iteration, const int max_branch_length,
      const int min_branch_length, const int goal_radius)
      : max_iteration_(max_iteration), max_branch_length_(max_branch_length),
        min_branch_length_(min_branch_length), goal_radius_(goal_radius)
  {
  }
  ~RRT() {}
  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;
  Log GetLog() override;

private:
  Log log_;

  std::vector<std::shared_ptr<NodeParent>> visited_nodes_;
  int max_iteration_{10000};
  int max_branch_length_{10};
  int min_branch_length_{5};
  int goal_radius_{5};
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_STAR_RRT_STAR_H_ */
