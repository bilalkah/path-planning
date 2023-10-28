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
#include <mutex>
#include <queue>

namespace planning
{
namespace tree_base
{

Path RRTStar::FindPath(const Node &start_node, const Node &goal_node,
                       const std::shared_ptr<Map> map)
{
  auto start_time{std::chrono::high_resolution_clock::now()};

  std::shared_ptr<NodeParent> final{std::nullptr_t()};
  auto current_cost = std::numeric_limits<double>::max();

  auto map_copy{std::make_shared<Map>(*map)};

  auto root{std::make_shared<NodeParent>(Node(start_node), nullptr, Cost{})};

  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_.first.clear();
    log_.first.push_back(root);
  }

  for (auto i = 0; i < max_iteration_; i++)
    {
      auto random_node{RandomNode(map_copy)};
      auto neighbor_vector =
          GetNearestNodeParentVector(neighbor_radius_, random_node, log_.first);
      auto new_node =
          WireNodeIfPossible(random_node, neighbor_vector, map_copy);

      if (new_node == nullptr)
        {
          continue;
        }

      new_node->cost =
          Cost(new_node->parent->cost.g + 1,
               new_node->parent->cost.h +
                   EuclideanDistance(new_node->node, new_node->parent->node));

      {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_.first.push_back(new_node);
      }

      parent_child_map_[new_node->parent].push_back(new_node);

      CheckIfGoalReached(new_node, final, goal_node);

      if (!neighbor_vector.empty())
        {
          auto is_rewired{Rewire(new_node, neighbor_vector, map_copy)};
          if (is_rewired && final != std::nullptr_t() &&
              current_cost > final->cost.f)
            {
              current_cost = final->cost.f;
              std::cout << "Goal cost: " << final->cost.f << "\t";

              auto end_time{std::chrono::high_resolution_clock::now()};
              std::cout
                  << "Time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         end_time - start_time)
                         .count()
                  << " ms" << std::endl;
            }
        }
    }
  return ReconstructPath(final);
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
      std::lock_guard<std::mutex> lock(log_mutex_);
      auto goal{std::make_shared<NodeParent>(
          goal_node, new_node,
          Cost(new_node->cost.g + 1, new_node->cost.h + remaining_distance))};

      if (final != std::nullptr_t())
        {
          if (goal->cost.f < final->cost.f)
            {
              final = goal;
              log_.first.push_back(final);
              std::cout << "Path changed." << std::endl;
            }
        }
      else
        {
          final = goal;
          log_.first.push_back(final);
          log_.second = final;
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

      auto new_cost{Cost(new_node->cost.g + 1,
                         new_node->cost.h +
                             EuclideanDistance(new_node->node, nearest->node))};

      if (new_cost.f < nearest->cost.f)
        {
          if (!CheckIfCollisionBetweenNodes(new_node->node, nearest->node, map))
            {
              parent_child_map_[nearest->parent].erase(
                  std::remove(parent_child_map_[nearest->parent].begin(),
                              parent_child_map_[nearest->parent].end(),
                              nearest),
                  parent_child_map_[nearest->parent].end());

              nearest->parent = new_node;
              nearest->cost = new_cost;
              parent_child_map_[new_node].push_back(nearest);
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

} // namespace tree_base
} // namespace planning
