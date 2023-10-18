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
#include <vector>

namespace planning
{
namespace tree_base
{

/**
 * @brief Rapidly-exploring Random Tree (RRT) algorithm.
 *
 */
class RRTStar : public IPlanningWithLogging
{
public:
  RRTStar() {}
  RRTStar(const int max_iteration) : max_iteration_(max_iteration) {}
  RRTStar(const int max_iteration, const int max_branch_length,
          const int min_branch_length, const int neighbor_radius,
          const int goal_radius, const int save_log_interval)
      : max_iteration_(max_iteration), max_branch_length_(max_branch_length),
        min_branch_length_(min_branch_length),
        neighbor_radius_(neighbor_radius), goal_radius_(goal_radius),
        save_log_interval_(save_log_interval)
  {
  }
  ~RRTStar() {}
  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;
  Log GetLog() override;
  std::vector<std::pair<Log, Path>> GetLogVector();

private:
  bool Rewire(const std::shared_ptr<NodeParent> &new_node,
              std::vector<std::shared_ptr<NodeParent>> &nearest_nodes,
              const std::shared_ptr<Map> map);
  void RecursivelyCostUpdate(const std::shared_ptr<NodeParent> &node);
  void SaveCurrentLogAndPath(const int iteration,
                             const std::shared_ptr<NodeParent> &final);
  void ResetLogAndUpdateWithVisitedNodes();

  Log log_;
  std::vector<std::pair<Log, Path>> log_vector_;
  std::vector<std::shared_ptr<NodeParent>> visited_nodes_;

  int max_iteration_{10000};
  int max_branch_length_{10};
  int min_branch_length_{5};
  int neighbor_radius_{20};
  int goal_radius_{5};
  int save_log_interval_{100};
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_RRT_H_ */
