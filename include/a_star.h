/**
 * @file a_star.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief A star path finding algorithm for 2D grid maps.
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "include/i_path_finding.h"
#include "include/utils.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <sys/types.h>
#include <utility>
#include <vector>

namespace planning
{

using SearchSpace = std::vector<std::pair<uint8_t, uint8_t>>;
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

class AStar : public IPathFinding
{
public:
  AStar(std::string direction);
  ~AStar();

  Path FindPath(const Node &start_node, const Node &goal_node,
                std::shared_ptr<Map> map) override;

private:
  bool InBounds(const Node &node, std::shared_ptr<Map> map);
  bool IsFree(const Node &node, std::shared_ptr<Map> map);
  bool IsGoal(const Node &node, const Node &goal_node);

  Path ReconstructPath(std::shared_ptr<NodeInfo> current_node);

  SearchSpace search_space_;
}; // class AStar
} // namespace planning