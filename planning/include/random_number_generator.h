/**
 * @file random_number_generator.cpp
 * @author Yusuf Etkin KIZILDAĞ (etkin866@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_TREE_BASE_RANDOM_NUMBER_GENERATOR_H_
#define PLANNING_TREE_BASE_RANDOM_NUMBER_GENERATOR_H_

#include <random>

/**
 * @brief Random number generator class for the integer values.
 *
 */
class RandomIntGenerator final
{
public:
  RandomIntGenerator(int const min, int const max) : gen_{std::random_device{}()}, dis_{min, max} {}
  int operator()() { return dis_(gen_); }

private:
  std::mt19937 gen_;
  std::uniform_int_distribution<> dis_;
};

#endif /* PLANNING_TREE_BASE_RANDOM_NUMBER_GENERATOR_H_ */