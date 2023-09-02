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

#include "test_fixture.h"
#include <gtest/gtest.h>

namespace planning
{

TEST_F(TestFixture, MapCreation)
{

  std::cout << "\n\n" << std::endl;

  auto occupied_count = 0u;
  for (auto i = 0; i < map_->GetHeight(); i++)
    {
      for (auto j = 0; j < map_->GetWidth(); j++)
        {
          if (map_->GetNodeState(Node(j, i)) == NodeState::kOccupied)
            {
              occupied_count++;
              std::cout << "0 ";
            }
          else
            {
              std::cout << "1 ";
            }
        }
      std::cout << std::endl;
    }
  std::cout << "\n\n" << std::endl;

  EXPECT_EQ(occupied_count, 17u)
      << "Occupied node count is not correct. Count result: " << occupied_count
      << std::endl;
}

} // namespace planning