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

#include "planning/include/data_types.h"

#include <memory>
#include <vector>

namespace planning
{
class Map;

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
  virtual Path FindPath(const Node &start_node, const Node &goal_node,
                        const std::shared_ptr<Map> map) = 0;

  virtual ~IPlanning() {}
}; // class IPathFinding

struct LogType
{
  LogType(Node current_node, Node parent_node, NodeState node_state)
      : current_node_(current_node), parent_node_(parent_node),
        node_state_(node_state)
  {
  }
  LogType(Node current_node, NodeState node_state)
      : current_node_(current_node), node_state_(node_state)
  {
  }
  Node current_node_;
  Node parent_node_;
  NodeState node_state_;
};

using Log = std::vector<LogType>;
class ILogging
{
public:
  virtual Log GetLog() = 0;

  virtual ~ILogging() {}
};

class IPlanningWithLogging : public IPlanning, public ILogging
{
};

} // namespace planning

#endif /* PLANNING_INCLUDE_I_PLANNING_H_ */
