/**
 * @file a_star.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "include/a_star.h"
#include "include/i_path_finding.h"

#include <cstddef>
#include <memory>
#include <sys/types.h>

#include <cstdint>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace planning
{

auto Compare = [](const std::shared_ptr<NodeInfo> &lhs,
                  const std::shared_ptr<NodeInfo> &rhs) {
  return lhs->cost.f > rhs->cost.f;
};

auto heuristic = [](const Node &lhs, const Node &rhs) {
  return std::abs(lhs.X() - rhs.X()) + std::abs(lhs.Y() - rhs.Y());
};

SearchSpace four_directions = {
    std::make_pair(0, 1),  // right
    std::make_pair(1, 0),  // down
    std::make_pair(0, -1), // left
    std::make_pair(-1, 0)  // up
};

SearchSpace eight_directions = {
    std::make_pair(0, 1),   // right
    std::make_pair(1, 0),   // down
    std::make_pair(0, -1),  // left
    std::make_pair(-1, 0),  // up
    std::make_pair(1, 1),   // down right
    std::make_pair(1, -1),  // down left
    std::make_pair(-1, -1), // up left
    std::make_pair(-1, 1)   // up right
};

AStar::AStar(std::string direction)
{
  if (direction == "four")
    {
      search_space_ = std::move(four_directions);
    }
  else if (direction == "eight")
    {
      search_space_ = std::move(eight_directions);
    }
  else
    {
      throw std::invalid_argument("Invalid direction argument.");
    }
};

AStar::~AStar(){};

Path AStar::FindPath(const Node &start_node, const Node &goal_node,
                     const std::shared_ptr<Map> map)
{
  // Copy map to avoid changing it.
  auto map_copy = std::make_shared<Map>(*map);

  std::priority_queue<std::shared_ptr<NodeInfo>,
                      std::vector<std::shared_ptr<NodeInfo>>, decltype(Compare)>
      open(Compare);

  auto start_node_info = std::make_shared<NodeInfo>(
      std::make_shared<Node>(start_node), nullptr, Cost(0, 0));

  open.push(start_node_info);
  map_copy->SetNodeState(start_node, NodeState::kVisited);

  while (!open.empty() && !IsGoal(*open.top()->node, goal_node))
    {
      auto current_node_info = open.top();
      open.pop();

      for (auto &direction : search_space_)
        {
          int x = current_node_info->node->X() + direction.first;
          int y = current_node_info->node->Y() + direction.second;

          if (!InBounds(Node(x, y), map_copy))
            {
              continue;
            }

          if (!IsFree(Node(x, y), map_copy))
            {
              continue;
            }

          auto neighbor_node_info = std::make_shared<NodeInfo>(
              std::make_shared<Node>(x, y), current_node_info,
              Cost(current_node_info->cost.g + 1,
                   heuristic(Node(x, y), goal_node)));

          open.push(neighbor_node_info);
          map_copy->SetNodeState(Node(x, y), NodeState::kVisited);
        }
    }

  if (open.empty())
    {
      std::cout << "No path found." << std::endl;
      return Path();
    }

  auto current_node_info = open.top();
  return ReconstructPath(current_node_info);
};

Path AStar::ReconstructPath(std::shared_ptr<NodeInfo> current_node)
{
  Path path;
  auto iterator_node = current_node;
  while (iterator_node->parent != nullptr)
    {
      path.push_front(iterator_node->node);
      iterator_node = iterator_node->parent;
    }
  return path;
};

bool AStar::InBounds(const Node &node, std::shared_ptr<Map> map)
{
  return node.X() >= 0 && node.X() < map->Width() && node.Y() >= 0 &&
         node.Y() < map->Height();
};

bool AStar::IsFree(const Node &node, std::shared_ptr<Map> map)
{
  return (*map)[node.X()][node.Y()] == NodeState::kFree;
};

bool AStar::IsGoal(const Node &node, const Node &goal_node)
{
  return node.X() == goal_node.X() && node.Y() == goal_node.Y();
};

} // namespace planning
