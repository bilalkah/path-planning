/**
 * @file test_bfs.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "grid_base/bfs/bfs.h"
#include "test_fixture.h"

namespace planning
{

using namespace planning::grid_base;

TEST_F(TestFixture, PathPlanning_WithBFS)
{
  constexpr int search_space{4};
  auto path_finder = std::make_shared<BFS>(search_space);
  const auto start_node = Node(1, 5);
  const auto goal_node = Node(7, 8);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  EXPECT_GT(path.size(), 0u) << " Path is not found.";
}

TEST_F(RealMapTestFixture, PathPlanningOnRealMap_WithBFS)
{
  constexpr int search_space{4};
  auto path_finder = std::make_shared<BFS>(search_space);
  const auto start_node = Node(90, 185);
  const auto goal_node = Node(445, 336);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  EXPECT_GT(path.size(), 0u) << "Path is not found";
  std::cout << "Path size: " << path.size() << std::endl;
}

} // namespace planning
