/**
 * @file rrt.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Rapidly-exploring Random Tree (RRT) algorithm.
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_TREE_BASE_RRT_RRT_H_
#define PLANNING_TREE_BASE_RRT_RRT_H_

#include "planning/include/i_planning.h"
#include "planning/tree_base/include/common_tree_base.h"

namespace planning
{
namespace tree_base
{

struct RRTStarCost
{
  RRTStarCost() : g(0), e(0), f(0) {}
  RRTStarCost(double g, double e) : g(g), e(e), f(g + e) {}

  double g; // cost from start node
  double e; // real distance from start node
  double f; // total cost
};

/**
 * @brief Rapidly-exploring Random Tree (RRT) algorithm.
 *
 */
class RRTStar : public IPlanningWithLogging
{
public:
  RRTStar() {}
  RRTStar(const int max_iteration) : max_iteration_(max_iteration) {}
  ~RRTStar() {}
  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;
  Log GetLog() override;

private:
  void
  Rewire(const std::shared_ptr<NodeParent<RRTStarCost>> &new_node,
         std::vector<std::shared_ptr<NodeParent<RRTStarCost>>> &nearest_nodes,
         const std::shared_ptr<Map> map);

  void ConvertVisitedNodesToLog(std::shared_ptr<NodeParent<RRTStarCost>> goal);
  void
  RecursivelyCostUpdate(const std::shared_ptr<NodeParent<RRTStarCost>> &node);

  Log log_;
  std::vector<std::shared_ptr<NodeParent<RRTStarCost>>> visited_nodes_;

  int max_iteration_{10000};
  int max_branch_length_{10};
  int min_branch_length_{5};
  int neighbor_radius_{20};
  int goal_radius_{5};
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_RRT_H_ */
