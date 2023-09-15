/**
 * @file node_parent.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_INCLUDE_NODE_PARENT_H_
#define PLANNING_INCLUDE_NODE_PARENT_H_

#include "planning/include/data_types.h"

namespace planning
{

/**
 * @brief Node with parent and cost.
 *
 * @tparam T Cost type.
 */
template <typename T> struct NodeParent
{
  NodeParent() : node(nullptr), parent(nullptr), cost(0) {}
  NodeParent(std::shared_ptr<Node> node_, std::shared_ptr<NodeParent> parent_)
      : node(node_), parent(parent_), cost(0)
  {
  }
  NodeParent(std::shared_ptr<Node> node_, std::shared_ptr<NodeParent> parent_,
             T cost_)
      : node(node_), parent(parent_), cost(cost_)
  {
  }
  std::shared_ptr<Node> node;
  std::shared_ptr<NodeParent> parent;
  T cost;
};

/**
 * @brief Backtrace path from goal to start.
 *
 */
template <typename T>
Path ReconstructPath(std::shared_ptr<NodeParent<T>> current_node)
{
  Path path;
  auto iterator_node = current_node;
  while (iterator_node->parent != nullptr)
    {
      path.push_back(iterator_node->node);
      iterator_node = iterator_node->parent;
    }
  return path;
}

} // namespace planning

#endif // PLANNING_INCLUDE_NODE_PARENT_H_
