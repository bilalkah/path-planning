/**
 * @file test_ray_cast.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/tree_base/include/common_tree_base.h"

#include <gtest/gtest.h>

namespace planning
{
namespace tree_base
{

TEST(UnitTest, IfRayCastWorks)
{
  Node node1{2, 7};
  Node node2{5, 9};

  auto ray{Get2DRayBetweenNodes(node1, node2)};

  ASSERT_EQ(ray.size(), 5);
}

} // namespace tree_base
} // namespace planning