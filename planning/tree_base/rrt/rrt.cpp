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

RRT::RRT(int const min, int const max, int const max_iter_num, int const max_step_size, double const goal_tolerance)
    : log_{}, max_iterations_{max_iter_num}, max_step_size_{max_step_size}, goal_tolerance_{goal_tolerance},
      rng_{min, max}
{
}

// add node to tree
[[nodiscard]] auto RRT::AddNode(const Node &node) -> bool
{
  tree_.push_back(node);
  return true;
}

[[nodiscard]] auto RRT::FindPath(const Node &start_node, const Node &goal_node, std::shared_ptr<Map> map) -> Path
{
  auto const adding_node_success{AddNode(start_node)};
  if (adding_node_success)
    {
      int iter_num{0};
      while (iter_num < max_iterations_ || !IsGoal(tree_.back(), goal_node))
        {
          const auto random_node{rng_()};
          const auto nearest_node{GetNearestNode(random_node)};
          const auto new_node{GetNewNode(nearest_node, random_node)};
          if (IsNodeValid(new_node, map))
            {
              auto const success{AddNode(new_node)};
              if (success)
                {
                  if (IsGoal(new_node, goal_node))
                    {
                      //   return GetPath(new_node);
                    }
                }
            }
          iter_num++;
        }
    }
  return Path{};
}

[[nodiscard]] auto RRT::GetNearestNode(const Node &node) -> Node
{
  return *std::min_element(std::cbegin(tree_), std::cend(tree_), [node](const Node &a, const Node &b) {
    return GetDistance(node, a) < GetDistance(node, b);
  });
}

[[nodiscard]] auto RRT::GetNewNode(const Node &nearest_node, const Node &random_node) -> Node
{
  const auto distance{GetDistance(nearest_node, random_node)};
  const auto step_size{std::min(distance, max_step_size_)};
  const auto theta{std::atan2(random_node.y - nearest_node.y, random_node.x - nearest_node.x)};
  return Node{nearest_node.x + step_size * std::cos(theta), nearest_node.y + step_size * std::sin(theta)};
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