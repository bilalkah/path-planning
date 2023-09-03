/**
 * @file i_planning.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Path planning interface.
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_INCLUDE_I_PLANNING_H_
#define PLANNING_INCLUDE_I_PLANNING_H_

#include <memory>
#include <vector>

namespace planning
{
class Map;
class Node;

class IPlanning
{
public:
  /**
   * @brief Find path between start and goal nodes.
   *
   * @param start_node Start node.
   * @param goal_node Goal node.
   * @param map Map to find path on.
   */
  virtual std::vector<std::shared_ptr<Node>>
  FindPath(const Node &start_node, const Node &goal_node,
           const std::shared_ptr<Map> map) = 0;
}; // class IPathFinding
} // namespace planning

#endif /* PLANNING_INCLUDE_I_PLANNING_H_ */
