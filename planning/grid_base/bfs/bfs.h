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
#include <string>

namespace planning
{
namespace grid_base
{

using CostBFS = int;

class BFS : public IPlanning
{

public:
  BFS() : search_space_(SearchSpaceGenerator().eight_directions) {}
  BFS(std::string search_space);
  ~BFS() {}

  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;

private:
  SearchSpace search_space_;
};

} // namespace grid_base
} // namespace planning

#endif /* PLANNING_GRID_BASE_BFS_BFS_H_ */