/**
 * @file test_acces_data.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/grid_base/include/common_grid_base.h"

#include <fstream>
#include <iostream>
#include <string>
#include <gtest/gtest.h>

TEST(test, AccessData_ReadMap)
{
  std::string dataDirectory = DATA_DIR;
  std::string dataFilePath = dataDirectory + "/bg2/AR0072SR.map";
  planning::Map map(dataFilePath);

  EXPECT_EQ(map.GetHeight(), 512);
  EXPECT_EQ(map.GetWidth(), 512);
}
