/**
 * @file common_planning.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/include/common_planning.h"

namespace planning
{
// Map
Map::Map(int height, int width) : height_(height), width_(width)
{
  map_.resize(height_);
  for (auto &row : map_)
    {
      row.resize(width_);
      std::fill(row.begin(), row.end(), NodeState::kOccupied);
    }
}
Map::Map(std::string &file_path)
{
  std::ifstream file(file_path);
  std::string line;
  std::getline(file, line);

  std::getline(file, line);
  height_ = std::stoi(line.substr(line.find(" ") + 1));

  std::getline(file, line);
  width_ = std::stoi(line.substr(line.find(" ") + 1));

  std::getline(file, line);

  map_.resize(height_);
  for (auto &row : map_)
    {
      row.resize(width_);
      std::getline(file, line);
      for (auto i = 0; i < width_; i++)
        {
          if (line[i] == '.' || line[i] == 'G')
            {
              row[i] = NodeState::kFree;
            }
          else
            {
              row[i] = NodeState::kOccupied;
            }
        }
    }
}
int Map::GetWidth() const { return width_; }
int Map::GetHeight() const { return height_; }
NodeState Map::GetNodeState(const Node &node) const
{
  return map_[node.x][node.y];
}
void Map::SetNodeState(const Node &node, NodeState node_state)
{
  map_[node.x][node.y] = node_state;
}
void Map::Visualize() const
{
  for (auto i = 0; i < height_; i++)
    {
      for (auto j = 0; j < width_; j++)
        {
          std::cout << static_cast<int>(map_[i][j]) << " ";
        }
      std::cout << std::endl;
    }
  std::cout << "\n" << std::endl;
}
void Map::UpdateMapWithPath(const Path &path)
{
  for (const auto &node : path)
    {
      map_[node->x][node->y] = NodeState::kPath;
    }
}

bool IsInbound(const Node &node, const std::shared_ptr<Map> map)
{
  return node.x >= 0 && node.x < map->GetHeight() && node.y >= 0 &&
         node.y < map->GetWidth();
}

bool IsFree(const Node &node, const std::shared_ptr<Map> map)
{
  return IsInbound(node, map) && (map->GetNodeState(node) == NodeState::kFree ||
                                  map->GetNodeState(node) == NodeState::kGoal ||
                                  map->GetNodeState(node) == NodeState::kStart);
}

bool IsGoal(const Node &node, const Node &goal_node)
{
  return node.x == goal_node.x && node.y == goal_node.y;
}

} // namespace planning
