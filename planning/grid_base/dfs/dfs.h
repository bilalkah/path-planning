/**
 * @file dfs.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief  Depth First Search algorithm.
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_GRID_BASE_DFS_DFS_H_
#define PLANNING_GRID_BASE_DFS_DFS_H_

#include "utility/common_grid_base.h"
#include "utility/i_planning.h"
#include <mutex>
#include <stack>
#include <string>

namespace planning
{

namespace grid_base
{

/**
 * @brief Depth First Search algorithm.
 *
 */
class DFS : public IPlanningWithLogging
{
public:
  DFS(const int search_space);
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
  Log log_{};
  SearchSpace search_space_{};

  std::mutex log_mutex_{};
};

} // namespace grid_base
} // namespace planning

#endif // PLANNING_GRID_BASE_DFS_DFS_H_
