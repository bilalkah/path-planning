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

#include "include/a_star.h"
#include "include/utils.h"
#include <cstddef>
#include <gtest/gtest.h>
#include <memory>

class TestFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    srand(time(NULL));
    Createmap(10, 10, 5);
  }

  virtual void TearDown() {}

  void Createmap(size_t width, size_t height, size_t occupied_nodes)
  {
    map_ = std::make_shared<Map>(width, height);

    // Fill map with random occupied nodes.
    for (size_t i = 0; i < occupied_nodes; i++)
      {
        size_t x = rand() % width;
        size_t y = rand() % height;
        (*map_)[y][x] = NodeState::kOccupied;
      }
  }

  std::shared_ptr<Map> map_;
};

#endif /* TESTS_HELPER_H_ */
