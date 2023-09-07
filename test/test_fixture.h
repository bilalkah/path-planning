/**
 * @file test_fixture.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Fixture for tests.
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TEST_TEST_FIXTURE_H_
#define TEST_TEST_FIXTURE_H_

#include "planning/grid_base/include/common_grid_base.h"
#include <cstddef>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>

using namespace planning;

class RealMapTestFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dataDirectory = DATA_DIR;
    std::string dataFilePath = dataDirectory + "/bg2/AR0072SR.map";

    map_ = std::make_shared<Map>(dataFilePath);
  }
  virtual void TearDown() {}

  std::shared_ptr<Map> map_;
};

class TestFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    map_data_ = "11111111111000011111111101111111110"
                "11111111100001111111110111101111000"
                "110111111111011111111111111111";
    srand(time(NULL));
    Createmap(10, 10);
  }

  virtual void TearDown() {}

  void Createmap(size_t width, size_t height)
  {
    map_ = std::make_shared<Map>(width, height);

    // fill the map according to the map data
    for (size_t i = 0; i < map_data_.size(); i++)
      {
        if (map_data_[i] == '1')
          {
            map_->SetNodeState(Node(i / width, i % width), NodeState::kFree);
          }
      }
  }

  void PrintMap()
  {
    std::cout << "\n\n" << std::endl;
    for (auto i = 0u; i < map_->GetHeight(); i++)
      {
        for (auto j = 0u; j < map_->GetWidth(); j++)
          {
            if (map_->GetNodeState(Node(j, i)) == NodeState::kOccupied)
              {
                std::cout << "0 ";
              }
            else if (map_->GetNodeState(Node(j, i)) == NodeState::kFree)
              {
                std::cout << "1 ";
              }
          }
        std::cout << std::endl;
      }
    std::cout << "\n\n" << std::endl;
  }

  std::shared_ptr<Map> map_;
  std::string map_data_;
};

#endif /* TEST_TEST_FIXTURE_H_ */
