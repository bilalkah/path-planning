/**
 * @file common.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Common definitions for planning module.
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_COMMON_H_
#define PLANNING_COMMON_H_

#include <iostream>
#include <iterator>
#include <memory>
#include <vector>

namespace planning
{

/**
 * @brief Simple Node class.
 *
 */
class Node
{
public:
  Node() : x_(0), y_(0) {}
  Node(int x, int y) : x_(x), y_(y) {}
  ~Node() {}

  int X() const { return x_; }
  int Y() const { return y_; }

  friend std::ostream &operator<<(std::ostream &os, const Node &node)
  {
    os << "Node: " << node.x_ << " " << node.y_;
    return os;
  }

private:
  int x_, y_;
}; // class Node

/**
 * @brief Path type.
 *
 */
using Path = std::vector<std::shared_ptr<Node>>;

/**
 * @brief Node with parent and cost.
 *
 * @tparam T Cost type.
 */
template <typename T> struct NodeParent
{
  NodeParent() : node(nullptr), parent(nullptr) {}
  NodeParent(std::shared_ptr<Node> node_, std::shared_ptr<NodeParent> parent_)
      : node(node_), parent(parent_)
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
 * @brief Map class that holds the map data in a 2D vector.
 *
 */
class Map
{
public:
  Map(int height, int width) : height_(height), width_(width)
  {
    map_.resize(height_);
    for (auto &row : map_)
      {
        row.resize(width_);
        std::fill(row.begin(), row.end(), NodeState::kFree);
      }
  }
  ~Map() {}

  int GetWidth() const { return width_; }
  int GetHeight() const { return height_; }

  /**
   * @brief Get the Node State object of cell. Does not check for out of bounds.
   *
   * @param node
   * @return NodeState
   */
  NodeState GetNodeState(const Node &node) const
  {
    return map_[node.X()][node.Y()];
  }

  /**
   * @brief Set the Node State object of cell. Does not check for out of
   * bounds.
   *
   * @param node
   * @return NodeState
   */
  void SetNodeState(const Node &node, NodeState node_state)
  {
    map_[node.X()][node.Y()] = node_state;
  }

  /**
   * @brief Visualize map.
   *
   */
  void Visualize() const
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

  /**
   * @brief Visualize map with path.
   *
   */
  void UpdateMapWithPath(const Path &path)
  {
    for (const auto &node : path)
      {
        map_[node->X()][node->Y()] = NodeState::kPath;
      }
  }

private:
  int height_, width_;
  std::vector<std::vector<NodeState>> map_;
}; // class Map

/**
 * @brief Inbound check for map.
 *
 */

inline bool IsInbound(const Node &node, const std::shared_ptr<Map> map)
{
  return node.X() >= 0 && node.X() < map->GetHeight() && node.Y() >= 0 &&
         node.Y() < map->GetWidth();
}

/**
 * @brief Check if node is free.
 *
 */

inline bool IsFree(const Node &node, const std::shared_ptr<Map> map)
{
  return IsInbound(node, map) && (map->GetNodeState(node) == NodeState::kFree ||
                                  map->GetNodeState(node) == NodeState::kGoal ||
                                  map->GetNodeState(node) == NodeState::kStart);
}

/**
 * @brief Check if node is goal.
 *
 */
inline bool IsGoal(const Node &node, const Node &goal_node)
{
  return node.X() == goal_node.X() && node.Y() == goal_node.Y();
}

/**
 * @brief Backtrace path from goal to start.
 *
 */
template <typename T>
static Path ReconstructPath(std::shared_ptr<NodeParent<T>> current_node)
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
#endif /* PLANNING_COMMON_H_ */