/**
 * @file data_types.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Common data types used in the project.
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_DATA_TYPES_H_
#define INCLUDE_DATA_TYPES_H_

#include <list>
#include <cstdint>
#include <memory>
#include <vector>

namespace planning
{

enum class NodeState : uint8_t
{
  kFree,
  kVisited,
  kOccupied,
  kStart,
  kGoal,
  kPath
}; // enum class NodeState

class Node
{
public:
  Node() : x_(0), y_(0) {}
  Node(int x, int y) : x_(x), y_(y) {}
  ~Node() {}

  int X() const { return x_; }
  int Y() const { return y_; }

private:
  int x_, y_;
}; // class Node

struct Cost
{
  Cost() : g(0), h(0), f(0) {}
  Cost(int g_, int h_) : g(g_), h(h_), f(g_ + h_) {}

  int g; // cost of step from start to this node
  int h; // heuristic cost of this node
  int f; // total cost of this node: f = g + h
};

struct NodeInfo
{
  NodeInfo() : node(nullptr), parent(nullptr), cost() {}
  NodeInfo(std::shared_ptr<Node> node_, std::shared_ptr<NodeInfo> parent_,
           Cost cost_)
      : node(node_), parent(parent_), cost(cost_)
  {
  }

  std::shared_ptr<Node> node;
  std::shared_ptr<NodeInfo> parent;
  Cost cost;
};

} // namespace planning

#endif /* INCLUDE_DATA_TYPES_H_ */