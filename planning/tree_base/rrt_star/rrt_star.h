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

#include "utility/common_tree_base.h"
#include "utility/i_planning.h"
#include <memory>
#include <mutex>
#include <unordered_map>
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
          const int goal_radius)
      : max_iteration_(max_iteration), max_branch_length_(max_branch_length),
        min_branch_length_(min_branch_length),
        neighbor_radius_(neighbor_radius), goal_radius_(goal_radius)
  {
  }
  ~RRTStar() {}
  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;
  Log GetLog() override
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return log_;
  }
  void ClearLog() override
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_.first.clear();
    log_.second = nullptr;
  }

private:
  std::shared_ptr<NodeParent>
  WireNodeIfPossible(const Node &random_node,
                     std::vector<std::shared_ptr<NodeParent>>,
                     const std::shared_ptr<Map> map);
  void CheckIfGoalReached(const std::shared_ptr<NodeParent> &new_node,
                          std::shared_ptr<NodeParent> &final,
                          const Node &goal_node);
  bool Rewire(const std::shared_ptr<NodeParent> &new_node,
              std::vector<std::shared_ptr<NodeParent>> &nearest_nodes,
              const std::shared_ptr<Map> map);
  void IterativelyCostUpdate(const std::shared_ptr<NodeParent> &node);

  Log log_;
  std::unordered_map<std::shared_ptr<NodeParent>,
                     std::vector<std::shared_ptr<NodeParent>>>
      parent_child_map_{};

  int max_iteration_{10000};
  int max_branch_length_{10};
  int min_branch_length_{5};
  int neighbor_radius_{15};
  int goal_radius_{5};
  int save_log_interval_{100};
  std::mutex log_mutex_;
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_RRT_H_ */
