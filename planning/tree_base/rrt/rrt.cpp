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
#include <cstddef>
#include <iostream>

namespace planning
{
namespace tree_base
{

Path RRT::FindPath(const Node &start_node, const Node &goal_node,
                   const std::shared_ptr<Map> map)
{
  log_.clear();
  visited_nodes_.clear();

  // Copy map to avoid changing it.
  auto map_copy{std::make_shared<Map>(*map)};

  // Set start node as kStart.
  map_copy->SetNodeState(start_node, NodeState::kStart);

  // Create root node.
  auto root{std::make_shared<NodeParent<RRTCost>>(
      std::make_shared<Node>(start_node), nullptr, RRTCost())};

  // push start node to visited nodes.
  visited_nodes_.emplace_back(root);

  for (auto i = 0; i < max_iteration_; i++)
    {
      // Get random node from random point according to map.
      auto random_node{RandomNode(map_copy)};

      // Get nearest node.
      auto nearest_node{GetNearestNodeParent(random_node, visited_nodes_)};

      // Wire new node to nearest node if there is no collision.
      auto new_node{WireNewNode(max_branch_length_, min_branch_length_,
                                random_node, nearest_node, map_copy)};

      // Check if new node is valid.
      if (new_node == nullptr)
        {
          continue;
        }

      // Update cost of new node.
      new_node->cost =
          RRTCost(nearest_node->cost.g + 1,
                  nearest_node->cost.e +
                      EuclideanDistance(*new_node->node, *nearest_node->node));

      // Add new node to visited nodes.
      visited_nodes_.emplace_back(new_node);
      log_.emplace_back(
          LogType{*new_node->node, *nearest_node->node, NodeState::kVisited});

      // Set nodes between new node and nearest node as kVisited.
      auto ray{Get2DRayBetweenNodes(*nearest_node->node, *new_node->node)};
      for (const auto &node : ray)
        {
          map_copy->SetNodeState(node, NodeState::kVisited);
        }

      // Check if goal node is in radius.
      if (EuclideanDistance(*new_node->node, goal_node) <= goal_radius_)
        {
          // Add goal node to visited nodes.
          auto goal{std::make_shared<NodeParent<RRTCost>>(
              std::make_shared<Node>(goal_node), new_node,
              RRTCost(new_node->cost.g + 1,
                      new_node->cost.e +
                          EuclideanDistance(*new_node->node, goal_node)))};
          log_.emplace_back(
              LogType{*goal->node, *new_node->node, NodeState::kGoal});

          // Get path.
          return ReconstructPath(goal);
        }
    }

  return Path();
}

Log RRT::GetLog() { return log_; }

} // namespace tree_base
} // namespace planning
