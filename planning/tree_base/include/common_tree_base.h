/**
 * @file common_tree_base.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_
#define PLANNING_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_

#include "planning/include/common_planning.h"
#include "planning/include/i_planning.h"
#include <cstddef>
#include <limits>
#include <memory>

namespace planning
{
namespace tree_base
{

/**
 * @brief // Random sampling function (x, y) between (0, 1)
 *
 */
std::pair<double, double> RandomSampling();

/**
 * @brief Random node from map
 *
 * @param map
 * @return Node
 */
Node RandomNode(const std::shared_ptr<Map> map);

/**
 * @brief // Get 2D ray between two nodes
 *
 * @param node1
 * @param node2
 * @return std::vector<Node>
 */
std::vector<Node> Get2DRayBetweenNodes(const Node &node1, const Node &node2);

/**
 * @brief // Euclidean distance between two nodes
 *
 * @param node1
 * @param node2
 * @return double
 */
double EuclideanDistance(const Node &node1, const Node &node2);

/**
 * @brief Check if there is collision between two nodes
 *
 * @param node1
 * @param node2
 * @param map
 * @return true if there is collision
 * @return false if there is no collision
 */
bool CheckIfCollisionBetweenNodes(const Node &node1, const Node &node2,
                                  const std::shared_ptr<Map> map);

/**
 * @brief Get the Nearest Node Parent object
 *
 * @tparam T
 * @param node
 * @param nodes
 * @return std::shared_ptr<NodeParent<T>>
 */
template <typename T>
std::shared_ptr<NodeParent<T>>
GetNearestNodeParent(const Node &node,
                     const std::vector<std::shared_ptr<NodeParent<T>>> &nodes)
{
  std::shared_ptr<NodeParent<T>> nearest_node;
  double min_distance{std::numeric_limits<double>::max()};

  for (const auto &node_parent : nodes)
    {
      double distance{EuclideanDistance(*node_parent->node, node)};
      if (distance < min_distance)
        {
          min_distance = distance;
          nearest_node = node_parent;
        }
    }

  return nearest_node;
}

/**
 * @brief Get the Nearest Node Parent Vector object
 *
 * @tparam T
 * @param neighbor_radius
 * @param node
 * @param nodes
 * @return std::vector<std::shared_ptr<NodeParent<T>>>
 */
template <typename T>
std::vector<std::shared_ptr<NodeParent<T>>> GetNearestNodeParentVector(
    const int neighbor_radius, const Node &node,
    const std::vector<std::shared_ptr<NodeParent<T>>> &nodes)
{
  std::vector<std::shared_ptr<NodeParent<T>>> nearest_nodes;

  // Search in the neighbor_radius
  for (const auto &node_parent : nodes)
    {
      double distance{EuclideanDistance(*node_parent->node, node)};
      if (distance < neighbor_radius)
        {
          nearest_nodes.emplace_back(node_parent);
        }
    }

  return nearest_nodes;
}

/**
 * @brief Wire new node to nearest node if there is no collision.
 *
 * @tparam T
 * @param max_branch_length
 * @param min_branch_length
 * @param random_node
 * @param nearest_node
 * @param map
 * @return std::shared_ptr<NodeParent<T>>
 */
template <typename T>
std::shared_ptr<NodeParent<T>>
WireNewNode(const int max_branch_length, const int min_branch_length,
            const Node &random_node,
            const std::shared_ptr<NodeParent<T>> &nearest_node,
            const std::shared_ptr<Map> map)
{
  // Calculate distance between random node and nearest node.
  double distance{EuclideanDistance(random_node, *nearest_node->node)};
  // Calculate unit vector between random node and nearest node.
  double unit_vector_x{(random_node.x_ - nearest_node->node->x_) / distance};
  double unit_vector_y{(random_node.y_ - nearest_node->node->y_) / distance};

  // Calculate new node.
  Node new_node{random_node};
  if (distance > max_branch_length)
    {
      new_node.x_ = nearest_node->node->x_ + max_branch_length * unit_vector_x;
      new_node.y_ = nearest_node->node->y_ + max_branch_length * unit_vector_y;
      distance = EuclideanDistance(new_node, *nearest_node->node);
    }

  // Check if there is collision between new node and nearest node.
  // If there is no collision, wire new node to nearest node.
  // If there is collision, check closer nodes to nearest node.
  // Until there is no collision or reach to nearest node.
  int i{0};

  while (new_node != *nearest_node->node &&
         CheckIfCollisionBetweenNodes(new_node, *nearest_node->node, map))
    {
      // Calculate new node closer to nearest node by 1 unit.
      new_node.x_ -= unit_vector_x;
      new_node.y_ -= unit_vector_y;
      if (i < min_branch_length)
        {
          return std::nullptr_t();
        }
      i += 1;
    }

  // Check if new node is valid.
  if (new_node != *nearest_node->node &&
      map->GetNodeState(new_node) != NodeState::kOccupied)
    {
      return std::make_shared<NodeParent<T>>(std::make_shared<Node>(new_node),
                                             nearest_node, T());
    }

  return std::nullptr_t();
};

} // namespace tree_base
} // namespace planning

#endif // PLANNING_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_