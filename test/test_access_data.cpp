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

#include "planning/common_planning.h"

#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <string>

TEST(test, AccessData)
{
  std::string dataDirectory = DATA_DIR;
  std::string dataFilePath = dataDirectory + "/README.md";

  EXPECT_EQ("/home/bilal/mint_training/maps/README.md", dataFilePath);
}

TEST(test, AccessData_ReadMap)
{
  std::string dataDirectory = DATA_DIR;
  std::string dataFilePath = dataDirectory + "/bg2/AR0072SR.map";

  EXPECT_EQ("/home/bilal/mint_training/maps/bg2/AR0072SR.map", dataFilePath);

  planning::Map map(dataFilePath);

  EXPECT_EQ(map.GetHeight(), 512);
  EXPECT_EQ(map.GetWidth(), 512);
}
