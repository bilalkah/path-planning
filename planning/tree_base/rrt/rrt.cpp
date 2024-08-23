/**
 * @file rrt_star.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/tree_base/rrt/rrt.h"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <mutex>
#include <thread>
namespace planning
{
namespace tree_base
{

Path RRT::FindPath(const Node &start_node, const Node &goal_node,
                   const std::shared_ptr<Map> map)
{
  auto map_copy{std::make_shared<Map>(*map)};
  map_copy->SetNodeState(start_node, NodeState::kStart);

  auto root{std::make_shared<NodeParent>(start_node, nullptr, Cost{})};
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_.first.clear();
    log_.second = nullptr;
    log_.first.emplace_back(root);
  }

  for (auto i = 0; i < max_iteration_; i++)
    {
      auto random_node{RandomNode(map_copy)};
      auto nearest_node{GetNearestNodeParent(random_node, log_.first)};
      auto new_node{WireNewNode(max_branch_length_, min_branch_length_,
                                random_node, nearest_node, map_copy)};

      if (new_node == nullptr)
        {
          continue;
        }

      new_node->cost =
          Cost(nearest_node->cost.g + 1,
               nearest_node->cost.h +
                   EuclideanDistance(new_node->node, nearest_node->node));

      {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_.first.emplace_back(new_node);
      }
      map_copy->SetNodeState(new_node->node, NodeState::kVisited);

      // Check if goal node is in radius.
      if (EuclideanDistance(new_node->node, goal_node) <= goal_radius_)
        {
          // Add goal node to visited nodes.
          auto goal{std::make_shared<NodeParent>(
              goal_node, new_node,
              Cost(new_node->cost.g + 1,
                   new_node->cost.h +
                       EuclideanDistance(new_node->node, goal_node)))};
          {
            std::lock_guard<std::mutex> lock(log_mutex_);
            log_.first.emplace_back(goal);
            log_.second = goal;
          }

          // Get path.
          return ReconstructPath(goal);
        }
    }

  return Path();
}

} // namespace tree_base
} // namespace planning
