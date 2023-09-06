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

#include "planning/include/node_parent.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

namespace planning
{

/**
 * @brief Map class that holds the map data in a 2D vector.
 *
 */
class Map
{
public:
  Map(int, int);
  Map(std::string &);
  ~Map() {}

  int GetWidth() const;
  int GetHeight() const;

  /**
   * @brief Get the Node State object of cell. Does not check for out of bounds.
   *
   * @param node
   * @return NodeState
   */
  NodeState GetNodeState(const Node &) const;

  /**
   * @brief Set the Node State object of cell. Does not check for out of
   * bounds.
   *
   * @param node
   * @return NodeState
   */
  void SetNodeState(const Node &, NodeState);

  /**
   * @brief Visualize map.
   *
   */
  void Visualize() const;

  /**
   * @brief Visualize map with path.
   *
   */
  void UpdateMapWithPath(const Path &);

private:
  int height_, width_;
  std::vector<std::vector<NodeState>> map_;
}; // class Map

/**
 * @brief Inbound check for map.
 *
 */
bool IsInbound(const Node &, const std::shared_ptr<Map>);

/**
 * @brief Check if node is free.
 *
 */
bool IsFree(const Node &, const std::shared_ptr<Map>);

/**
 * @brief Check if node is goal.
 *
 */
bool IsGoal(const Node &, const Node &);

/**
 * @brief Return distance between two nodes.
 *
 */
double GetDistance(const Node &, const Node &);

} // namespace planning
#endif /* PLANNING_INCLUDE_COMMON_PLANNING_H_ */
