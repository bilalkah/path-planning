/**
 * @file a_star.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief A* path finding algorithm.
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_GRID_BASE_A_STAR_A_STAR_H_
#define PLANNING_GRID_BASE_A_STAR_A_STAR_H_

#include "planning/grid_base/include/common_grid_base.h"
#include "planning/include/i_planning.h"

#include <memory>
#include <string>
#include <vector>

namespace planning
{

namespace grid_base
{

struct Cost
{
  Cost() : g(0), h(0), f(0) {}
  Cost(double g, double h) : g(g), h(h), f(g + h) {}

  double g; // cost from start node
  double h; // heuristic cost to goal node
  double f; // total cost
};

/**
 * @brief A* path finding algorithm.
 *
 */
template <typename SearchSpace> class AStar : public IPlanning
{
public:
  AStar(const SearchSpace &search_space) : search_space_{search_space} {}
  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;
  Log GetLog() override;
private:
  SearchSpace search_space_;
  ::planning::Log log_;
}; // class AStar

template class AStar<Directions4>;
template class AStar<Directions8>;

} // namespace grid_base

} // namespace planning

#endif /* PLANNING_GRID_BASE_A_STAR_A_STAR_H_ */
