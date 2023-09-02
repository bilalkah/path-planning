/**
 * @file bfs.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Breadth first search path finding algorithm for 2D grid maps.
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_BFS_H_
#define INCLUDE_BFS_H_

#include "include/i_path_finding.h"
#include "include/utils.h"

namespace planning
{



struct NodeInfo
{
  NodeInfo() : node(nullptr), parent(nullptr) {}
  NodeInfo(std::shared_ptr<Node> node_, std::shared_ptr<NodeInfo> parent_)
      : node(node_), parent(parent_)
  {
  }

  std::shared_ptr<Node> node;
  std::shared_ptr<NodeInfo> parent;
};

class BFS : public IPathFinding
{
public:
  BFS(std::string direction);
  ~BFS();

  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;

private:
  bool InBounds(const Node &node, std::shared_ptr<Map> map);
  bool IsFree(const Node &node, std::shared_ptr<Map> map);
  bool IsGoal(const Node &node, const Node &goal_node);

  Path ReconstructPath(std::shared_ptr<NodeInfo> current_node);

  SearchSpace search_space_;
}; // class BFS

} // namespace planning

#endif // INCLUDE_BFS_H_
