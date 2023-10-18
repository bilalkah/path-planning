/**
 * @file a_star.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "a_star.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <queue>

namespace planning
{
namespace grid_base
{

auto Compare{
    [](std::shared_ptr<NodeParent> lhs, std::shared_ptr<NodeParent> rhs) {
      return lhs->cost.f > rhs->cost.f;
    }};

// Euclidean distance.
auto heuristic{[](const Node &lhs, const Node &rhs) {
  return std::hypot(lhs.x_ - rhs.x_, lhs.y_ - rhs.y_);
}};

AStar::AStar(const double &heuristic_weight, const int search_space)
    : heuristic_weight_(heuristic_weight)
{

  if (search_space == 4)
    {
      search_space_ = GetFourDirection();
    }
  else if (search_space == 8)
    {
      search_space_ = GetEightDirection();
    }
  else
    {
      std::cout << "Invalid search space." << std::endl;
    }
}

Path AStar::FindPath(const Node &start_node, const Node &goal_node,
                     const std::shared_ptr<Map> map)
{
  // Clear log.
  log_.clear();
  log_.emplace_back(LogType{start_node, NodeState::kStart});
  log_.emplace_back(LogType{goal_node, NodeState::kGoal});
  // Copy map to avoid changing it.
  std::shared_ptr<Map> map_copy = std::make_shared<Map>(*map);
  map_copy->SetNodeState(goal_node, NodeState::kGoal);

  // Create priority queue for search list.
  std::priority_queue<std::shared_ptr<NodeParent>,
                      std::vector<std::shared_ptr<NodeParent>>,
                      decltype(Compare)>
      search_list(Compare);

  // Create start node.
  auto start_node_info = std::make_shared<NodeParent>(
      start_node, nullptr,
      Cost(0, heuristic(start_node, goal_node), heuristic_weight_));

  // Add start node to search list.
  search_list.push(start_node_info);

  while (!search_list.empty() && !IsGoal(search_list.top()->node, goal_node))
    {
      // Get the node with the lowest cost.
      auto current_node = search_list.top();
      search_list.pop();

      if (!IsFree(current_node->node, map_copy))
        {
          continue;
        }
      log_.emplace_back(LogType{current_node->node, NodeState::kVisited});
      // Update map.
      map_copy->SetNodeState(current_node->node, NodeState::kVisited);

      // Check neighbors.
      for (auto &direction : search_space_)
        {
          int x = current_node->node.x_ + direction[0];
          int y = current_node->node.y_ + direction[1];

          if (!IsFree(Node(x, y), map_copy))
            {
              continue;
            }

          auto neighbor_node = std::make_shared<NodeParent>(
              Node(x, y), current_node,
              Cost(current_node->cost.g + 1, heuristic(Node(x, y), goal_node),
                   heuristic_weight_));

          search_list.push(neighbor_node);
        }
    }

  if (search_list.empty())
    {
      std::cout << "No path found." << std::endl;
      return Path{};
    }

  auto current_node = search_list.top();
  auto path = ReconstructPath(current_node);
  map_copy->UpdateMapWithPath(path);
  return path;
}

Log AStar::GetLog() { return log_; }

} // namespace grid_base
} // namespace planning
