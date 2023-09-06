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

#include "../../include/common_planning.h"
#include "../../include/data_types.h"
#include "../../include/node_parent.h"
#include "../include/random_node_generator.h"
#include <algorithm>

namespace planning
{
namespace tree_base
{

using RRTSteps = int;
/**
 * @brief Rapidly-exploring Random Tree (RRT) algorithm.
 *
 */

class RRT
{
public:
  /**
   * @brief Construct a new RRT object.
   *
   */
  RRT(int const, int const, int const, int const, double const);

  /**
   * @brief Find path from start node to goal node.
   *
   * @param start_node Start node.
   * @param goal_node Goal node.
   * @param map Map to search.
   * @return Path Path from start node to goal node.
   */
  [[nodiscard]] auto FindPath(const Node &, const Node &, std::shared_ptr<Map>) -> Path;

  /**
   * @brief Get log of the algorithm.
   *
   * @return Log of the algorithm.
   */
  [[nodiscard]] auto GetLog() const -> Log;

  // add node to tree
  [[nodiscard]] auto AddNode(const Node &) -> bool;
  [[nodiscard]] auto GetNearestNode(const Node &) -> Node;
  [[nodiscard]] auto GetNewNode(const Node &, const Node &) -> Node;
  [[nodiscard]] auto IsNodeValid(const Node &, std::shared_ptr<Map>) -> bool;
  [[nodiscard]] auto IsGoal(const Node &, const Node &) -> bool;

private:
  // tree
  std::vector<Node> tree_{};
  //   std::vector<NodeParent<RRTSteps>> tree_{};

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
