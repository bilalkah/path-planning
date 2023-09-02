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

#include "planning/grid_base/bfs/bfs.h"
#include "test_fixture.h"

namespace planning
{

using namespace planning::grid_base;

TEST_F(TestFixture, PathPlanning_WithBFS)
{
  auto path_finder = std::make_shared<BFS>("four");
  const auto start_node = Node(1, 5);
  const auto goal_node = Node(7, 8);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  auto expected_path_size = 18u;
  // expect path size be greater than expected size and if not print actual size
  // and expected size.
  std::cout << "Actual path size: " << path.size()
            << " Expected path size: " << expected_path_size << std::endl;

  EXPECT_GE(path.size(), expected_path_size)
      << "Actual path size: " << path.size()
      << " Expected path size: " << expected_path_size;

  for (const auto &node : path)
    {
      std::cout << node->X() << " " << node->Y() << std::endl;
    }
}

} // namespace planning