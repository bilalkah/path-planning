/**
 * @file dfs.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "dfs.h"
#include "common_grid_base.h"
#include "common_planning.h"
#include <iostream>
#include <stdexcept>

namespace planning
{
namespace grid_base
{

DFS::DFS(const int search_space)
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

Path DFS::FindPath(const Node &start_node, const Node &goal_node,
                   const std::shared_ptr<Map> map)
{
  // Clear log.
  log_.clear();
  log_.emplace_back(LogType{start_node, NodeState::kStart});
  log_.emplace_back(LogType{goal_node, NodeState::kGoal});
  // Copy map to avoid changing it.
  std::shared_ptr<Map> map_copy = std::make_shared<Map>(*map);
  map_copy->SetNodeState(goal_node, NodeState::kGoal);

  // Create stack for search list.
  std::stack<std::shared_ptr<NodeParent<CostDFS>>> search_list;

  // Create start node.
  std::shared_ptr<NodeParent<CostDFS>> start_node_info =
      std::make_shared<NodeParent<CostDFS>>(std::make_shared<Node>(start_node),
                                            nullptr, CostDFS{0});
  search_list.push(start_node_info);

  while (!search_list.empty() && !IsGoal(*search_list.top()->node, goal_node))
    {
      auto current_node = search_list.top();
      search_list.pop();

      // Check if node is already visited.
      if (map_copy->GetNodeState(*current_node->node) == NodeState::kVisited)
        {
          continue;
        }
      log_.emplace_back(LogType{*current_node->node, NodeState::kVisited});
      // Set node state to visited.
      map_copy->SetNodeState(*current_node->node, NodeState::kVisited);

      for (const auto &direction : search_space_)
        {
          int x = current_node->node->x_ + direction[0];
          int y = current_node->node->y_ + direction[1];

          if (!IsFree(Node(x, y), map_copy))
            {
              continue;
            }

          // Create new node.
          std::shared_ptr<NodeParent<CostDFS>> new_node_parent =
              std::make_shared<NodeParent<CostDFS>>(
                  std::make_shared<Node>(Node(x, y)), current_node,
                  CostDFS{current_node->cost + 1});

          // Add new node to search list.
          search_list.push(new_node_parent);
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

Log DFS::GetLog() { return log_; }

} // namespace grid_base
} // namespace planning
