/**
 * @file rrt.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Rapidly-exploring Random Tree (RRT) algorithm.
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_TREE_BASE_RRT_RRT_H_
#define PLANNING_TREE_BASE_RRT_RRT_H_

#include "planning/include/common_planning.h"
#include "planning/include/data_types.h"
#include "planning/include/node_parent.h"
#include "planning/tree_base/include/random_node_generator.h"
#include "planning/tree_base/rrt/rrt_tree/rrt_tree.h"
#include <algorithm>

namespace planning
{
namespace tree_base
{

using RRTSteps = int;

class RRT
{
public:
  RRT(int const, int const, int const, double const, double const);
  [[nodiscard]] auto FindPath(const Node &, const Node &, std::shared_ptr<Map>) -> Path;
  [[nodiscard]] auto GetLog() const -> Log;
  [[nodiscard]] auto AddNode(const Node &, const Node &, Cost const) -> bool;
  [[nodiscard]] auto GetNearestNode(const Node &) -> Node;
  [[nodiscard]] auto GetNewNode(const Node &, const Node &) -> Node;
  [[nodiscard]] auto IsNodeValid(const Node &, std::shared_ptr<Map>) -> bool;
  [[nodiscard]] auto IsGoal(const Node &, const Node &) -> bool;

private:
  RRTTree tree_;

  // log of the algorithm
  Log log_{};
  int max_iterations_{1};
  double max_step_size_{1.0};
  double goal_tolerance_{0.1};
  RandomNodeGenerator rng_{0, 1};
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_RRT_H_ */
