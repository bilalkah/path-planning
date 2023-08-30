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
#include "i_path_finding.h"
#include "test_fixture.h"
#include "utils.h"
#include <gtest/gtest.h>
#include <memory>

namespace planning
{
TEST_F(TestFixture, PathPlanning_WithAStar)
{
  std::shared_ptr<IPathFinding> path_finder = std::make_shared<AStar>("four");
  const auto start_node = Node(0, 0);
  const auto goal_node = Node(9, 9);
  Path path = path_finder->FindPath(start_node, goal_node, map_);

  auto expected_path_size = 18u;
  // expect path size be greater than expected size and if not print actual size
  // and expected size.
  std::cout << "Actual path size: " << path.size()
            << " Expected path size: " << expected_path_size << std::endl;
            
  EXPECT_GE(path.size(), expected_path_size)
      << "Actual path size: " << path.size()
      << " Expected path size: " << expected_path_size;
}
} // namespace planning
