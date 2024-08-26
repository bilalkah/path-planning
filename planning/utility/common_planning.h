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

#ifndef PLANNING_INCLUDE_COMMON_PLANNING_H_
#define PLANNING_INCLUDE_COMMON_PLANNING_H_

#include "node_parent.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace planning
{

/**
 * @brief Map class that holds the map data in a 2D vector.
 *
 */
class Map
{
public:
  Map(std::size_t height, std::size_t width);
  Map(std::string &file_path);
  ~Map() {}

  std::size_t GetWidth() const;
  std::size_t GetHeight() const;

  /**
   * @brief Get the Node State object of cell. Does not check for out of bounds.
   *
   * @param node
   * @return NodeState
   */
  NodeState GetNodeState(const Node &node) const;

  /**
   * @brief Set the Node State object of cell. Does not check for out of
   * bounds.
   *
   * @param node
   * @return NodeState
   */
  void SetNodeState(const Node &node, NodeState node_state);

  /**
   * @brief Visualize map.
   *
   */
  void Visualize() const;

  /**
   * @brief Visualize map with path.
   *
   */
  void UpdateMapWithPath(const Path &path);

private:
  std::size_t height_, width_;
  std::vector<std::vector<NodeState>> map_;
}; // class Map

/**
 * @brief Inbound check for map.
 *
 */
bool IsInbound(const Node &node, const std::shared_ptr<Map> map);

/**
 * @brief Check if node is free.
 *
 */
bool IsFree(const Node &node, const std::shared_ptr<Map> map);

/**
 * @brief Check if node is goal.
 *
 */
bool IsGoal(const Node &node, const Node &goal_node);

} // namespace planning
#endif /* PLANNING_INCLUDE_COMMON_PLANNING_H_ */
