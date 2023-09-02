/**
 * @file bfs.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "include/bfs.h"
#include "i_path_finding.h"
#include "utils.h"

#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>

namespace planning
{



BFS::BFS(std::string direction)
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
      throw std::invalid_argument("Invalid direction argument");
    }
}

BFS::~BFS() {}

Path BFS::FindPath(const Node &start_node, const Node &goal_node,
                   const std::shared_ptr<Map> map)
{
  std::queue<std::shared_ptr<NodeInfo>> frontier;
  auto map_copy = std::make_shared<Map>(*map);
  auto start_node_info =
      std::make_shared<NodeInfo>(std::make_shared<Node>(start_node), nullptr);
  frontier.push(start_node_info);
  map_copy->SetNodeState(start_node, NodeState::kVisited);

  while (!frontier.empty() && !IsGoal(*frontier.front()->node, goal_node))
    {
      auto current_node_info = frontier.front();
      frontier.pop();

      for (auto &direction : search_space_)
        {
          auto new_node = std::make_shared<Node>(
              current_node_info->node->X() + direction.first,
              current_node_info->node->Y() + direction.second);

          if (InBounds(*new_node, map_copy) && IsFree(*new_node, map_copy))
            {
              auto new_node_info =
                  std::make_shared<NodeInfo>(new_node, current_node_info);

              frontier.push(new_node_info);
              map_copy->SetNodeState(*new_node, NodeState::kVisited);
            }
        }
    }

  if (frontier.empty())
    {
      std::cout << "No path found." << std::endl;
      return Path();
    }
  return ReconstructPath(frontier.front());
}

Path BFS::ReconstructPath(std::shared_ptr<NodeInfo> current_node)
{
  Path path;
  auto iterator_node = current_node;
  while (iterator_node->parent != nullptr)
    {
      path.push_front(iterator_node->node);
      iterator_node = iterator_node->parent;
    }
  return path;
}

bool BFS::InBounds(const Node &node, std::shared_ptr<Map> map)
{
  return node.X() >= 0 && node.X() < map->Height() && node.Y() >= 0 &&
         node.Y() < map->Width();
}

bool BFS::IsFree(const Node &node, std::shared_ptr<Map> map)
{
  return (*map)[node.X()][node.Y()] == NodeState::kFree;
}

bool BFS::IsGoal(const Node &node, const Node &goal_node)
{
  return node.X() == goal_node.X() && node.Y() == goal_node.Y();
}

} // namespace planning