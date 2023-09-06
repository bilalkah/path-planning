/**
 * @file data_types.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_INCLUDE_DATA_TYPES_H_
#define PLANNING_INCLUDE_DATA_TYPES_H_

#include <iostream>
#include <memory>
#include <vector>

namespace planning
{

/**
 * @brief Simple Node class.
 *
 */
struct Node
{
  Node() : x{0}, y{0} {}
  Node(int x, int y) : x{x}, y{y} {}

  friend std::ostream &operator<<(std::ostream &os, const Node &node)
  {
    os << "Node: " << node.x << " " << node.y;
    return os;
  }

  int x, y;
}; // class Node

/**
 * @brief Node state.
 *
 */
enum class NodeState : uint8_t
{
  kFree,
  kVisited,
  kOccupied,
  kStart,
  kGoal,
  kPath
}; // enum class NodeState

/**
 * @brief Path type.
 *
 */
using Path = std::vector<std::shared_ptr<Node>>;
using Log = std::vector<std::pair<Node, NodeState>>;

} // namespace planning

#endif // PLANNING_INCLUDE_DATA_TYPES_H_
