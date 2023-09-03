/**
 * @file bfs.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bfs.h"
#include "common_planning.h"
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>

namespace planning
{
namespace grid_base
{

template<typename SearchSpace>
Path BFS<SearchSpace>::FindPath(const Node &start_node, const Node &goal_node,
                   const std::shared_ptr<Map> map)
{
  // Copy map to avoid changing it.
  std::shared_ptr<Map> map_copy = std::make_shared<Map>(*map);
  map_copy->SetNodeState(goal_node, NodeState::kGoal);
  // Create queue for search list.
  std::queue<std::shared_ptr<NodeParent<CostBFS>>> search_list;

  // Create start node.
  auto start_node_parent = std::make_shared<NodeParent<CostBFS>>(
      std::make_shared<Node>(start_node), nullptr, CostBFS{0});

  // Add start node to search list.
  search_list.push(start_node_parent);

  while (!search_list.empty() &&
         !IsGoal(*(search_list.front()->node), goal_node))
    {
      // Get first node from search list.
      auto current_node_parent = search_list.front();
      search_list.pop();

      if (!IsFree(*current_node_parent->node, map_copy))
        {
          continue;
        }

      // Update map.
      map_copy->SetNodeState(*current_node_parent->node, NodeState::kVisited);

      for (const auto &direction : search_space_)
        {
          int x = current_node_parent->node->x_ + direction.first;
          int y = current_node_parent->node->y_ + direction.second;

          if (!IsFree(Node(x, y), map_copy))
            {
              continue;
            }

          auto neighbor_node = std::make_shared<NodeParent<CostBFS>>(
              std::make_shared<Node>(Node(x, y)), current_node_parent,
              CostBFS{current_node_parent->cost + 1});

          // Add neighbor node to search list.
          search_list.push(neighbor_node);
        }
    }
  if (search_list.empty())
    {
      std::cout << "No path found." << std::endl;
      return Path{};
    }

  auto current_node = search_list.front();
  auto path = ReconstructPath(current_node);
  map_copy->UpdateMapWithPath(path);

  return path;
}

} // namespace grid_base
} // namespace planning
