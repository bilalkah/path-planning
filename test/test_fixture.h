/**
 * @file helper.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Helper functions for tests.
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TESTS_HELPER_H_
#define TESTS_HELPER_H_

#include "planning/common_planning.h"
#include <cstddef>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>

using namespace planning;
class TestFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    map_data_ =
        "1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 0 "
        "1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 1 0 1 1 1 1 0 1 1 1 1 0 0 0 "
        "1 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 ";
    srand(time(NULL));
    Createmap(10, 10);
  }

  virtual void TearDown() {}

  void Createmap(size_t width, size_t height)
  {
    map_ = std::make_shared<Map>(width, height);

    // fill the map according to the map data
    std::istringstream iss(map_data_);
    std::vector<int> map_data((std::istream_iterator<int>(iss)),
                              std::istream_iterator<int>());
    for (size_t i = 0; i < map_data.size(); i++)
      {
        if (map_data[i] == 0)
          {
            map_->SetNodeState(Node(i % width, i / width),
                               NodeState::kOccupied);
          }
      }
  }

  void PrintMap()
  {
    std::cout << "\n\n" << std::endl;
    for (auto i = 0; i < map_->GetHeight(); i++)
      {
        for (auto j = 0; j < map_->GetWidth(); j++)
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

#endif /* TESTS_HELPER_H_ */
