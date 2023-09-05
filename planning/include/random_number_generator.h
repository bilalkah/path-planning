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

#include <random>

/**
 * @brief Random number generator class for the integer values.
 *
 */
template <int min, int max> class RandomIntGenerator final
{
public:
  RandomIntGenerator() : gen_{std::random_device{}()}, dis_{min, max} {}
  int operator()() { return dis_(gen_); }

private:
  std::mt19937 gen_;
  std::uniform_int_distribution<> dis_;
};