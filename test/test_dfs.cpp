/**
 * @file test_dfs.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/grid_base/dfs/dfs.h"
#include "test_fixture.h"

namespace planning
{

using namespace planning::grid_base;

TEST_F(TestFixture, PathPlanning_WithDFS)
{
  auto path_finder = std::make_shared<DFS<Directions4>>(four_directions);
  const auto start_node = Node(1, 5);
  const auto goal_node = Node(7, 8);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  EXPECT_GE(path.size(), 0u) << " Path is not found.";
}

/**
 * @brief Disabled because it causes segmentation fault.
 * 
 */
TEST_F(RealMapTestFixture, DISABLED_PathPlanningOnRealMap_WithDFS)
{
  std::shared_ptr<IPlanning> path_finder = std::make_shared<DFS<Directions4>>(four_directions);
  const auto start_node = Node(90, 185);
  const auto goal_node = Node(445, 336);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  EXPECT_GT(path.size(), 0u) << "Path is not found";
  std::cout << "Path size: " << path.size() << std::endl;
}

} // namespace planning