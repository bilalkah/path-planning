/**
 * @file common_tree_base.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/tree_base/include/common_tree_base.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
namespace planning
{
namespace tree_base
{

std::pair<double, double> RandomSampling()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  return std::make_pair(dis(gen), dis(gen));
}

Node RandomNode(const std::shared_ptr<Map> map)
{
  bool is_valid{false};
  Node random_node;
  while (!is_valid)
    {
      auto random_point{RandomSampling()};
      random_node =
          Node(static_cast<int>(random_point.first * map->GetWidth()),
               static_cast<int>(random_point.second * map->GetHeight()));
      if (map->GetNodeState(random_node) == NodeState::kFree)
        {
          is_valid = true;
        }
    }
  return random_node;
}

std::vector<Node> Get2DRayBetweenNodes(const Node &src, const Node &dst)
{
  std::pair<double, double> ray_vector{dst.x_ - src.x_, dst.y_ - src.y_};
  auto ray_length{std::hypot(ray_vector.first, ray_vector.second)};
  auto unit_vector{std::make_pair(ray_vector.first / ray_length,
                                  ray_vector.second / ray_length)};
  std::vector<Node> ray;
  if (src == dst)
    {
      return ray;
    }
  // Check ray vector corresponds which past of cartesian coordinate system.
  std::pair<double, double> sign_vector{0.5, 0.5};
  if (ray_vector.first < 0)
    {
      sign_vector.first = -0.5;
    }
  if (ray_vector.second < 0)
    {
      sign_vector.second = -0.5;
    }

  for (auto i = 0; i < ray_length; i++)
    {
      auto new_node_x{src.x_ + sign_vector.first + i * unit_vector.first};
      auto new_node_y{src.y_ + sign_vector.second + i * unit_vector.second};
      Node new_node{static_cast<int>(new_node_x), static_cast<int>(new_node_y)};
      ray.emplace_back(new_node);
    }

  // check first and last node.
  if (ray.front() != src)
    {
      ray.insert(ray.begin(), src);
    }
  if (ray.back() != dst)
    {
      ray.emplace_back(dst);
    }

  return ray;
}

double EuclideanDistance(const Node &node1, const Node &node2)
{
  return std::hypot(node1.x_ - node2.x_, node1.y_ - node2.y_);
}

bool CheckIfCollisionBetweenNodes(const Node &node1, const Node &node2,
                                  const std::shared_ptr<Map> map)
{
  auto ray{Get2DRayBetweenNodes(node1, node2)};
  if (ray.empty())
    {
      return true;
    }
  for (const auto &node : ray)
    {
      if (map->GetNodeState(node) == NodeState::kOccupied)
        {
          return true;
        }
    }

  return false;
}

} // namespace tree_base
} // namespace planning