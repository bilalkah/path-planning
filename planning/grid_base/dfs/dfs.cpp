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
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

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
  ClearLog();

  std::shared_ptr<Map> map_copy = std::make_shared<Map>(*map);
  map_copy->SetNodeState(goal_node, NodeState::kGoal);

  std::stack<std::shared_ptr<NodeParent>> search_list;

  std::shared_ptr<NodeParent> start_node_info =
      std::make_shared<NodeParent>(start_node, nullptr, Cost{});
  search_list.push(start_node_info);

  while (!search_list.empty() && !IsGoal(search_list.top()->node, goal_node))
    {
      auto current_node = search_list.top();
      search_list.pop();

      if (map_copy->GetNodeState(current_node->node) == NodeState::kVisited)
        {
          continue;
        }

      {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_.first.emplace_back(current_node);
      }
      map_copy->SetNodeState(current_node->node, NodeState::kVisited);

      for (const auto &direction : search_space_)
        {
          int x = current_node->node.x_ + direction[0];
          int y = current_node->node.y_ + direction[1];

          if (!IsFree(Node(x, y), map_copy))
            {
              continue;
            }

          std::shared_ptr<NodeParent> new_node_parent =
              std::make_shared<NodeParent>(Node(x, y), current_node, Cost{});

          search_list.push(new_node_parent);
        }
    }

  if (search_list.empty())
    {
      std::cout << "No path found." << std::endl;
      return Path{};
    }

  auto current_node = search_list.top();
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_.second = current_node;
  }
  auto path = ReconstructPath(current_node);
  map_copy->UpdateMapWithPath(path);
  return path;
}

} // namespace grid_base
} // namespace planning
