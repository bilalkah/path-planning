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

#include "planning/tree_base/rrt_star/rrt_star.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>

namespace planning
{
namespace tree_base
{

Path RRTStar::FindPath(const Node &start_node, const Node &goal_node,
                       const std::shared_ptr<Map> map)
{
  std::shared_ptr<NodeParent<RRTStarCost>> final = nullptr;
  bool is_goal_found{false};
  visited_nodes_.clear();
  log_.clear();

  // Copy map to avoid changing it.
  auto map_copy{std::make_shared<Map>(*map)};

  // Set start node as kStart.
  map_copy->SetNodeState(start_node, NodeState::kStart);

  // Create root node.
  auto root{std::make_shared<NodeParent<RRTStarCost>>(
      std::make_shared<Node>(start_node), nullptr, RRTStarCost())};

  // Add root node to visited nodes.
  visited_nodes_.push_back(root);

  for (auto i = 0; i < max_iteration_; i++)
    {
      // Random sampling.
      auto random_point{RandomSampling()};

      // Get random node from random point according to map.
      auto random_node{Node{
          static_cast<int>(random_point.first * (map_copy->GetHeight() - 1)),
          static_cast<int>(random_point.second * (map_copy->GetWidth() - 1))}};

      // Get all nearest nodes.
      auto neighbor_vector = GetNearestNodeParentVector(
          neighbor_radius_, random_node, visited_nodes_);

      std::shared_ptr<NodeParent<RRTStarCost>> new_node = std::nullptr_t();
      if (neighbor_vector.empty())
        {
          auto nearest_node{GetNearestNodeParent(random_node, visited_nodes_)};
          new_node = WireNewNode(max_branch_length_, min_branch_length_,
                                 random_node, nearest_node, map_copy);
        }
      else
        {
          auto index{0u};
          // Sort nearest nodes according to their cost from low to high.
          std::sort(neighbor_vector.begin(), neighbor_vector.end(),
                    [](const std::shared_ptr<NodeParent<RRTStarCost>> &node1,
                       const std::shared_ptr<NodeParent<RRTStarCost>> &node2) {
                      return node1->cost.f < node2->cost.f;
                    });

          while (new_node == nullptr && index < neighbor_vector.size())
            {
              new_node =
                  WireNewNode(max_branch_length_, min_branch_length_,
                              random_node, neighbor_vector[index], map_copy);
              index++;
            }
        }

      if (new_node == nullptr)
        {
          continue;
        }

      // Calculate cost.
      new_node->cost = RRTStarCost(
          new_node->parent->cost.g + 1,
          new_node->parent->cost.e +
              EuclideanDistance(*new_node->node, *new_node->parent->node));

      // Add new node to visited nodes.
      visited_nodes_.push_back(new_node);

      // Check if new node is close enough to goal node.
      if (EuclideanDistance(*new_node->node, goal_node) < goal_radius_)
        {
          // Create goal node.
          auto goal{std::make_shared<NodeParent<RRTStarCost>>(
              std::make_shared<Node>(goal_node), new_node,
              RRTStarCost(new_node->cost.g + 1,
                          new_node->cost.e +
                              EuclideanDistance(*new_node->node, goal_node)))};

          // Add goal node to visited nodes.
          visited_nodes_.push_back(goal);

          ConvertVisitedNodesToLog(goal);
          std::cout << "Goal found." << std::endl;
          std::cout << "Log size: " << log_.size() << std::endl;
          std::cout << "-----------" << std::endl;
          // Get path.
          is_goal_found = true;
          final = goal;
        }

      // Rewire.
      if (!neighbor_vector.empty())
        {
          Rewire(new_node, neighbor_vector, map_copy);
        }

      if (is_goal_found && i % 100 == 0)
        {
          ConvertVisitedNodesToLog(final);
        }
    }
  // ConvertVisitedNodesToLog();
  if (is_goal_found)
    {
      return ReconstructPath(final);
    }
  return Path();
}

Log RRTStar::GetLog() { return log_; }

void RRTStar::Rewire(
    const std::shared_ptr<NodeParent<RRTStarCost>> &new_node,
    std::vector<std::shared_ptr<NodeParent<RRTStarCost>>> &nearest_nodes,
    const std::shared_ptr<Map> map)
{
  for (const auto &nearest : nearest_nodes)
    {
      if (nearest->node == new_node->parent->node)
        {
          continue;
        }

      // Calculate new cost
      auto new_cost{
          RRTStarCost(new_node->cost.g + 1,
                      new_node->cost.e +
                          EuclideanDistance(*new_node->node, *nearest->node))};

      // Check if new cost is lower than old cost.
      if (new_cost.f < nearest->cost.f)
        {
          // Check if there is collision between new node and nearest node.
          if (!CheckIfCollisionBetweenNodes(*new_node->node, *nearest->node,
                                            map))
            {
              // Update parent.
              nearest->parent = new_node;

              // Update cost.
              nearest->cost = new_cost;

              RecursivelyCostUpdate(nearest);
            }
        }
    }
}

void RRTStar::RecursivelyCostUpdate(
    const std::shared_ptr<NodeParent<RRTStarCost>> &node)
{
  if (node == nullptr)
    {
      return;
    }
  if (node->parent == nullptr)
    {
      return;
    }

  for (auto &visited_node : visited_nodes_)
    {
      if (visited_node->parent == node)
        {
          visited_node->cost =
              RRTStarCost(visited_node->parent->cost.g + 1,
                          visited_node->parent->cost.e +
                              EuclideanDistance(*visited_node->node,
                                                *visited_node->parent->node));
          RecursivelyCostUpdate(visited_node);
        }
    }
}

void RRTStar::ConvertVisitedNodesToLog(
    std::shared_ptr<NodeParent<RRTStarCost>> goal)
{
  // Print Cost
  std::cout << "Cost: " << goal->cost.f << std::endl;
  auto path = ReconstructPath(goal);
  auto index{0u};
  for (index = 0; index < path.size() - 1; index++)
    {
      log_.emplace_back(
          LogType{*path[index], *path[index + 1], NodeState::kVisited});
    }
}

} // namespace tree_base
} // namespace planning
