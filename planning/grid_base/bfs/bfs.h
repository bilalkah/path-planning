/**
 * @file bfs.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Breadth First Search path finding algorithm.
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_GRID_BASE_BFS_BFS_H_
#define PLANNING_GRID_BASE_BFS_BFS_H_

#include "planning/grid_base/include/common_grid_base.h"
#include "planning/include/i_planning.h"
#include <cstdint>
#include <mutex>
#include <string>

namespace planning
{
namespace grid_base
{

/**
 * @brief Breadth First Search path finding algorithm.
 *
 */
class BFS : public IPlanningWithLogging
{

public:
  BFS(const int search_space);
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

#endif /* PLANNING_GRID_BASE_BFS_BFS_H_ */
