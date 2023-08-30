/**
 * @file i_path_finding.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Interface for path finding algorithms.
 * @version 0.1
 * @date 2023-08-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_I_PATH_FINDING_H_
#define INCLUDE_I_PATH_FINDING_H_

#include <list>
#include <memory>

class Map;
class Node;
using Path = std::list<std::shared_ptr<Node>>;
namespace planning
{
class IPathFinding
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
                        std::shared_ptr<Map> map) = 0;
}; // class IPathFinding
} // namespace planning

#endif /* INCLUDE_I_PATH_FINDING_H_ */