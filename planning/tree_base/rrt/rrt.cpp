/**
 * @file rrt.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt.h"

namespace planning
{
namespace tree_base
{

RRT::RRT(int const min, int const max, int const max_iter_num, double const max_step_size, double const goal_tolerance)
    : tree_{}, log_{}, max_iterations_{max_iter_num}, max_step_size_{max_step_size}, goal_tolerance_{goal_tolerance},
      rng_{min, max}
{
}

// add node to tree
[[nodiscard]] auto RRT::AddNode(const Node &node, const Node &nearestNode) -> bool
{
  return tree_.insertNode(std::make_shared<Node>(node), std::make_shared<Node>(nearestNode));
}

[[nodiscard]] auto RRT::FindPath(const Node &start_node, const Node &goal_node, std::shared_ptr<Map> map) -> Path
{
  tree_.setRoot(start_node);
  int iter_num{0};
  bool flag{false};
  while (iter_num < max_iterations_ && !flag)
  {
    const auto random_node{rng_()};
    const auto nearest_node{tree_.GetNearestNode(random_node)};
    const auto new_node{GetNewNode(nearest_node, random_node)};
    if (IsNodeValid(new_node, map))
    {
      auto const success{AddNode(new_node, nearest_node)};
      if (success)
      {
        if (IsGoal(new_node, goal_node))
        {
          flag = true;
        }
      }
    }
    iter_num++;
  }

  if (flag)
  {
    return ReconstructPath(tree_.getNodes().back());
  }
  return Path{};
}

[[nodiscard]] auto RRT::GetNewNode(const Node &nearest_node, const Node &random_node) -> Node
{
  const auto distance{GetDistance(nearest_node, random_node)};
  const auto step_size{std::min(distance, max_step_size_)};
  const auto theta{std::atan2(random_node.y - nearest_node.y, random_node.x - nearest_node.x)};
  return Node{static_cast<int>(nearest_node.x + step_size * std::cos(theta)),
              static_cast<int>(nearest_node.y + step_size * std::sin(theta))};
}

[[nodiscard]] auto RRT::IsNodeValid(const Node &node, std::shared_ptr<Map> map) -> bool
{
//   return map->IsNodeValid(node);
    return false;
}

[[nodiscard]] auto RRT::IsGoal(const Node &node, const Node &goal_node) -> bool
{
  return GetDistance(node, goal_node) < goal_tolerance_;
}

[[nodiscard]] auto RRT::GetLog() const -> Log { return log_; }

} // namespace tree_base
} // namespace planning