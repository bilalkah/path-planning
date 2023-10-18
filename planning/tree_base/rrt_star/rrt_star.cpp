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
#include <chrono>
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
  // Start time
  auto start_time{std::chrono::high_resolution_clock::now()};
  log_.clear();
  visited_nodes_.clear();
  std::shared_ptr<NodeParent> final{std::nullptr_t()};
  auto current_cost = std::numeric_limits<double>::max();
  int path_count{0};

  // Copy map to avoid changing it.
  auto map_copy{std::make_shared<Map>(*map)};

  // Set start node as kStart.
  map_copy->SetNodeState(start_node, NodeState::kStart);

  // Create root node.
  auto root{std::make_shared<NodeParent>(Node(start_node),
                                         nullptr, Cost{})};

  // Add root node to visited nodes.
  visited_nodes_.push_back(root);

  for (auto i = 0; i < max_iteration_; i++)
    {
      // Random sampling.
      auto random_node{RandomNode(map_copy)};

      // Get all nearest nodes.
      auto neighbor_vector = GetNearestNodeParentVector(
          neighbor_radius_, random_node, visited_nodes_);

      std::shared_ptr<NodeParent> new_node{std::nullptr_t()};
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
                    [](const std::shared_ptr<NodeParent> &node1,
                       const std::shared_ptr<NodeParent> &node2) {
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
      new_node->cost =
          Cost(new_node->parent->cost.g + 1,
               new_node->parent->cost.h +
                   EuclideanDistance(new_node->node, new_node->parent->node));

      // Add new node to visited nodes.
      visited_nodes_.push_back(new_node);
      log_.push_back(
          LogType{new_node->node, new_node->parent->node, NodeState::kVisited});

      // Check if new node is close enough to goal node.
      if (EuclideanDistance(new_node->node, goal_node) < goal_radius_)
        {
          // Create goal node.
          auto goal{std::make_shared<NodeParent>(
              goal_node, new_node,
              Cost(new_node->cost.g + 1,
                   new_node->cost.h +
                       EuclideanDistance(new_node->node, goal_node)))};

          if (final != std::nullptr_t())
            {
              if (goal->cost.f < final->cost.f)
                {
                  // Remove old goal node from visited nodes.
                  visited_nodes_.erase(std::remove(visited_nodes_.begin(),
                                                   visited_nodes_.end(), final),
                                       visited_nodes_.end());
                  // Remove old goal node from log.
                  log_.erase(std::remove_if(log_.begin(), log_.end(),
                                            [&final](const LogType &log_type) {
                                              return log_type.current_node_ ==
                                                     final->node;
                                            }),
                             log_.end());
                  final = goal;
                  visited_nodes_.push_back(final);
                  log_.push_back(LogType{final->node, final->parent->node,
                                         NodeState::kVisited});
                  std::cout << "Path changed." << std::endl;
                }
            }
          else
            {
              final = goal;
              visited_nodes_.push_back(final);
              log_.push_back(LogType{final->node, final->parent->node,
                                     NodeState::kVisited});
              std::cout << "Path found." << std::endl;
            }
        }

      // Rewire.
      if (!neighbor_vector.empty())
        {
          auto is_rewired{Rewire(new_node, neighbor_vector, map_copy)};
          if (is_rewired && final != std::nullptr_t() &&
              current_cost > final->cost.f)
            {
              current_cost = final->cost.f;
              std::cout << "Goal cost: " << final->cost.f << "\t";
              std::cout << "Iteration: " << ++path_count << "\t";
              // Print time
              auto end_time{std::chrono::high_resolution_clock::now()};
              std::cout
                  << "Time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         end_time - start_time)
                         .count()
                  << " ms" << std::endl;
              SaveCurrentLogAndPath(i, final);
              ResetLogAndUpdateWithVisitedNodes();
            }
        }
    }
  return ReconstructPath(final);
}

Log RRTStar::GetLog() { return log_; }

std::vector<std::pair<Log, Path>> RRTStar::GetLogVector()
{
  return log_vector_;
}

bool RRTStar::Rewire(const std::shared_ptr<NodeParent> &new_node,
                     std::vector<std::shared_ptr<NodeParent>> &nearest_nodes,
                     const std::shared_ptr<Map> map)
{
  bool rewired{false};
  for (const auto &nearest : nearest_nodes)
    {
      if (nearest->node == new_node->parent->node)
        {
          continue;
        }

      // Calculate new cost
      auto new_cost{Cost(new_node->cost.g + 1,
                         new_node->cost.h +
                             EuclideanDistance(new_node->node, nearest->node))};

      // Check if new cost is lower than old cost.
      if (new_cost.f < nearest->cost.f)
        {
          // Check if there is collision between new node and nearest node.
          if (!CheckIfCollisionBetweenNodes(new_node->node, nearest->node, map))
            {
              // Update parent.
              nearest->parent = new_node;

              // Update cost.
              nearest->cost = new_cost;

              RecursivelyCostUpdate(nearest);

              rewired = true;
            }
        }
    }
  return rewired;
}

void RRTStar::RecursivelyCostUpdate(const std::shared_ptr<NodeParent> &node)
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
              Cost(visited_node->parent->cost.g + 1,
                   visited_node->parent->cost.h +
                       EuclideanDistance(visited_node->node,
                                         visited_node->parent->node));
          RecursivelyCostUpdate(visited_node);
        }
    }
}

void RRTStar::SaveCurrentLogAndPath(const int iteration,
                                    const std::shared_ptr<NodeParent> &final)
{

  {
    std::pair<Log, Path> log_path_pair;
    for (const auto &visited_node : visited_nodes_)
      {
        if (visited_node->parent == nullptr)
          {
            continue;
          }
        log_path_pair.first.push_back(LogType{visited_node->node,
                                              visited_node->parent->node,
                                              NodeState::kVisited});
      }
    log_path_pair.second = ReconstructPath(final);
    log_vector_.push_back(log_path_pair);
  }
}

void RRTStar::ResetLogAndUpdateWithVisitedNodes()
{
  log_.clear();
  for (const auto &visited_node : visited_nodes_)
    {
      if (visited_node->parent == nullptr)
        {
          continue;
        }
      log_.push_back(LogType{visited_node->node, visited_node->parent->node,
                             NodeState::kVisited});
    }
}

} // namespace tree_base
} // namespace planning
