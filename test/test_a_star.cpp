/**
 * @file test_a_star.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/grid_base/a_star/a_star.h"
#include "test_fixture.h"
#include <gtest/gtest.h>

namespace planning
{

using namespace planning::grid_base;

TEST_F(TestFixture, PathPlanning_WithAStar)
{
  constexpr double heuristic{0.5};
  constexpr int search_space{4};
  std::shared_ptr<IPlanning> path_finder =
      std::make_shared<planning::grid_base::AStar>(heuristic, search_space);
  const auto start_node = Node(0, 9);
  const auto goal_node = Node(9, 0);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  EXPECT_GT(path.size(), 0u) << "Path is not found";
}

TEST_F(RealMapTestFixture, PathPlanningOnRealMap_WithAStar)
{
  constexpr double heuristic{0.5};
  constexpr int search_space{4};
  std::shared_ptr<IPlanning> path_finder =
      std::make_shared<planning::grid_base::AStar>(heuristic, search_space);
  const auto start_node = Node(90, 185);
  const auto goal_node = Node(445, 336);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  EXPECT_GT(path.size(), 0u) << "Path is not found";
  std::cout << "Path size: " << path.size() << std::endl;
}

} // namespace planning
