/**
 * @file test_map.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/grid_base/include/common_grid_base.h"
#include "test_fixture.h"
#include <gtest/gtest.h>

namespace planning
{

TEST_F(TestFixture, MapCreation)
{

  std::cout << "\n\n" << std::endl;

  auto map_copy = std::make_shared<Map>(*map_);
  auto occupied_count = 0u;

  for (auto i = 0; i < map_copy->GetHeight(); i++)
    {
      for (auto j = 0; j < map_copy->GetWidth(); j++)
        {
          if (map_copy->GetNodeState(Node(i, j)) == NodeState::kOccupied)
            {
              occupied_count++;
            }
        }
    }

  EXPECT_EQ(occupied_count, 17u)
      << "Occupied node count is not correct. Count result: " << occupied_count
      << std::endl;
}

} // namespace planning
