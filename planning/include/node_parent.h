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
 * @brief Common cost struct for planning algorithms.
 *
 */
struct Cost
{
  Cost() : g(0), h(0), f(0) {}
  Cost(double g, double h) : g(g), h(h), f(g + h) {}
  Cost(double g, double h, double w) : g(g), h(h), f((1 - w) * g + w * h) {}

  double g; // cost from start node
  double h; // Euclidean distance to goal node or previous node
  double f; // total cost
};

/**
 * @brief Node with parent and cost.
 *
 * @tparam T Cost type.
 */
struct NodeParent
{
  NodeParent() {}
  NodeParent(Node node_, std::shared_ptr<NodeParent> parent_)
      : node(node_), parent(parent_)
  {
  }
  NodeParent(Node node_, std::shared_ptr<NodeParent> parent_, Cost cost_)
      : node(node_), parent(parent_), cost(cost_)
  {
  }
  Node node{};
  std::shared_ptr<NodeParent> parent{};
  Cost cost{};
};

/**
 * @brief Backtrace path from goal to start.
 *
 */
inline Path ReconstructPath(std::shared_ptr<NodeParent> current_node)
{
  Path path;
  if (current_node == nullptr)
    {
      return path;
    }
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
