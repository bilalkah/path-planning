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

#include "data_types.h"
#include "node_parent.h"

#include <memory>
#include <utility>
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

using Log = std::pair<std::vector<std::shared_ptr<NodeParent>>,
                      std::shared_ptr<NodeParent>>;

class ILogging
{
public:
  virtual Log GetLog() = 0;
  virtual void ClearLog() = 0;

  virtual ~ILogging() {}
};

class IPlanningWithLogging : public IPlanning, public ILogging
{
};

} // namespace planning

#endif /* PLANNING_INCLUDE_I_PLANNING_H_ */
