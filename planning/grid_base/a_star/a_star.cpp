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
#include <stdexcept>

namespace planning
{
namespace grid_base
{

auto Compare = [](const std::shared_ptr<NodeParent<Cost>> &lhs,
                  const std::shared_ptr<NodeParent<Cost>> &rhs) {
  return lhs->cost.f > rhs->cost.f;
};

// Euclidean distance.
auto heuristic = [](const Node &lhs, const Node &rhs) {
  return std::sqrt(std::pow(lhs.X() - rhs.X(), 2) +
                   std::pow(lhs.Y() - rhs.Y(), 2));
};

AStar::AStar(std::string search_space)
{
  if (search_space == "four")
    {
      search_space_ = std::move(SearchSpaceGenerator().four_directions);
    }
  else if (search_space == "eight")
    {
      search_space_ = std::move(SearchSpaceGenerator().eight_directions);
    }
  else
    {
      throw std::invalid_argument("Invalid search space.");
    }
}

Path AStar::FindPath(const Node &start_node, const Node &goal_node,
                     const std::shared_ptr<Map> map)
{
  // Copy map to avoid changing it.
  std::shared_ptr<Map> map_copy = std::make_shared<Map>(*map);

  // Create priority queue for search list.
  std::priority_queue<std::shared_ptr<NodeParent<Cost>>,
                      std::vector<std::shared_ptr<NodeParent<Cost>>>,
                      decltype(Compare)>
      search_list(Compare);

  // Create start node.
  auto start_node_info = std::make_shared<NodeParent<Cost>>(
      std::make_shared<Node>(start_node), nullptr,
      Cost(0, heuristic(start_node, goal_node)));

  // Add start node to search list.
  search_list.push(start_node_info);

  // Update map.
  map_copy->SetNodeState(start_node, NodeState::kStart);

  while (!search_list.empty() && !IsGoal(*search_list.top()->node, goal_node))
    {
      // Get the node with the lowest cost.
      auto current_node = search_list.top();
      search_list.pop();

      // Update map.
      map_copy->SetNodeState(*current_node->node, NodeState::kVisited);

      // Check neighbors.
      for (auto &direction : search_space_)
        {
          int x = current_node->node->X() + direction.first;
          int y = current_node->node->Y() + direction.second;

          if (!IsFree(Node(x, y), map_copy))
            {
              continue;
            }

          auto neighbor_node = std::make_shared<NodeParent<Cost>>(
              std::make_shared<Node>(Node(x, y)), current_node,
              Cost(current_node->cost.g + 1, heuristic(Node(x, y), goal_node)));

          search_list.push(neighbor_node);

          // Update map.
          map_copy->SetNodeState(Node(x, y), NodeState::kVisited);
        }
    }

  if (search_list.empty())
    {
      std::cout << "No path found." << std::endl;
      return Path();
    }

  auto current_node = search_list.top();
  return ReconstructPath(current_node);
}

} // namespace grid_base
} // namespace planning