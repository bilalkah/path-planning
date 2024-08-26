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

#include "common_planning.h"
#include "i_planning.h"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

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
 * @return std::shared_ptr<NodeParent>
 */
std::shared_ptr<NodeParent>
GetNearestNodeParent(const Node &node,
                     const std::vector<std::shared_ptr<NodeParent>> &nodes);

/**
 * @brief Get the Nearest Node Parent Vector object
 *
 * @tparam T
 * @param neighbor_radius
 * @param node
 * @param nodes
 * @return std::vector<std::shared_ptr<NodeParent>>
 */
std::vector<std::shared_ptr<NodeParent>> GetNearestNodeParentVector(
    const int neighbor_radius, const Node &node,
    const std::vector<std::shared_ptr<NodeParent>> &nodes);

/**
 * @brief Wire new node to nearest node if there is no collision.
 *
 * @tparam T
 * @param max_branch_length
 * @param min_branch_length
 * @param random_node
 * @param nearest_node
 * @param map
 * @return std::shared_ptr<NodeParent>
 */
std::shared_ptr<NodeParent>
WireNewNode(const int max_branch_length, const int min_branch_length,
            const Node &random_node,
            const std::shared_ptr<NodeParent> &nearest_node,
            const std::shared_ptr<Map> map);

} // namespace tree_base
} // namespace planning

#endif // PLANNING_TREE_BASE_INCLUDE_COMMON_TREE_BASE_H_