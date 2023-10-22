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
#include "common_tree_base.h"
#include "node_parent.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>

namespace planning
{
namespace tree_base
{

Path RRTStar::FindPath(const Node &start_node, const Node &goal_node,
                       const std::shared_ptr<Map> map)
{
  // Start timer
  auto start_time{std::chrono::high_resolution_clock::now()};

  // Clear log and visited nodes.
  log_.clear();
  visited_nodes_.clear();

  // Initialize final node.
  std::shared_ptr<NodeParent> final{std::nullptr_t()};
  auto current_cost = std::numeric_limits<double>::max();
  int path_count{0};

  // Copy map to avoid changing it.
  auto map_copy{std::make_shared<Map>(*map)};

  // Create root node.
  auto root{std::make_shared<NodeParent>(Node(start_node), nullptr, Cost{})};

  // Add root node to visited nodes.
  visited_nodes_.push_back(root);

  for (auto i = 0; i < max_iteration_; i++)
    {
      // Random sampling.
      auto random_node{RandomNode(map_copy)};

      // Get all nearest nodes.
      auto neighbor_vector = GetNearestNodeParentVector(
          neighbor_radius_, random_node, visited_nodes_);

      auto new_node =
          WireNodeIfPossible(random_node, neighbor_vector, map_copy);

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
      parent_child_map_[new_node->parent].push_back(new_node);
      AddVisitedLine(new_node, map_copy);
      log_.push_back(
          LogType{new_node->node, new_node->parent->node, NodeState::kVisited});

      // Check if new node is close enough to goal node.
      CheckIfGoalReached(new_node, final, goal_node);

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

      // Visualize.
      if (visualize_ != nullptr && i % 20 == 0)
        {
          visualize_->Visualize(visited_nodes_, final);
        }
    }
  return ReconstructPath(final);
}

Log RRTStar::GetLog() { return log_; }

std::vector<std::pair<Log, Path>> RRTStar::GetLogVector()
{
  return log_vector_;
}

std::shared_ptr<NodeParent> RRTStar::WireNodeIfPossible(
    const Node &random_node,
    std::vector<std::shared_ptr<NodeParent>> neighbor_vector,
    const std::shared_ptr<Map> map)
{
  std::shared_ptr<NodeParent> new_node{std::nullptr_t()};
  auto neighbor_iterator = neighbor_vector.begin();
  while (new_node == nullptr && neighbor_iterator != neighbor_vector.end())
    {
      new_node = WireNewNode(max_branch_length_, min_branch_length_,
                             random_node, (*neighbor_iterator), map);
      neighbor_iterator++;
    }

  return new_node;
}

void RRTStar::CheckIfGoalReached(const std::shared_ptr<NodeParent> &new_node,
                                 std::shared_ptr<NodeParent> &final,
                                 const Node &goal_node)
{
  auto remaining_distance{EuclideanDistance(new_node->node, goal_node)};
  if (remaining_distance < goal_radius_)
    {
      // Create goal node.
      auto goal{std::make_shared<NodeParent>(
          goal_node, new_node,
          Cost(new_node->cost.g + 1, new_node->cost.h + remaining_distance))};

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
          log_.push_back(
              LogType{final->node, final->parent->node, NodeState::kVisited});
          std::cout << "Path found." << std::endl;
        }
    }
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
              // Remove old parent from parent child map.
              parent_child_map_[nearest->parent].erase(
                  std::remove(parent_child_map_[nearest->parent].begin(),
                              parent_child_map_[nearest->parent].end(),
                              nearest),
                  parent_child_map_[nearest->parent].end());

              RemoveVisitedLine(nearest, map);

              // Update parent.
              nearest->parent = new_node;
              // Update cost.
              nearest->cost = new_cost;
              AddVisitedLine(nearest, map);
              // Add new parent to parent child map.
              parent_child_map_[new_node].push_back(nearest);
              // Update node's cost bfs
              IterativelyCostUpdate(nearest);

              rewired = true;
            }
        }
    }
  return rewired;
}

void RRTStar::IterativelyCostUpdate(const std::shared_ptr<NodeParent> &node)
{
  std::queue<std::shared_ptr<NodeParent>> queue;
  queue.push(node);

  while (!queue.empty())
    {
      auto current_node{queue.front()};
      queue.pop();

      for (const auto &child : parent_child_map_[current_node])
        {
          child->cost =
              Cost(child->parent->cost.g + 1,
                   child->parent->cost.h +
                       EuclideanDistance(child->node, child->parent->node));
          queue.push(child);
        }
    }
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

void RRTStar::AddVisitedLine(const std::shared_ptr<NodeParent> node,
                             const std::shared_ptr<Map> map_copy)
{
  auto ray{Get2DRayBetweenNodes(node->node, node->parent->node)};
  for (const auto &node : ray)
    {
      map_copy->SetNodeState(node, NodeState::kVisited);
    }
}

void RRTStar::RemoveVisitedLine(const std::shared_ptr<NodeParent> node,
                                const std::shared_ptr<Map> map_copy)
{
  auto ray{Get2DRayBetweenNodes(node->node, node->parent->node)};
  auto iterator = ray.begin() + 1;
  while (iterator != ray.end() - 1)
    {
      map_copy->SetNodeState(*iterator, NodeState::kFree);
      iterator++;
    }
}

} // namespace tree_base
} // namespace planning
