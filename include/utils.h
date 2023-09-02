/**
 * @file utils.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Utilies for path finding algorithms like map and node.
 * @version 0.1
 * @date 2023-08-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include "data_types.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

namespace planning
{

using SearchSpace = std::vector<std::pair<uint8_t, uint8_t>>;

SearchSpace four_directions = {
    std::make_pair(0, 1),  // right
    std::make_pair(1, 0),  // down
    std::make_pair(0, -1), // left
    std::make_pair(-1, 0)  // up
};

SearchSpace eight_directions = {
    std::make_pair(0, 1),   // right
    std::make_pair(1, 0),   // down
    std::make_pair(0, -1),  // left
    std::make_pair(-1, 0),  // up
    std::make_pair(1, 1),   // down right
    std::make_pair(1, -1),  // down left
    std::make_pair(-1, -1), // up left
    std::make_pair(-1, 1)   // up right
};

class Map
{
public:
  Map() : height_(0), width_(0){};
  Map(int height, int width) : height_(height), width_(width)
  {
    map_.resize(height_);
    for (auto &row : map_)
      {
        row.resize(width_);
        std::fill(row.begin(), row.end(), NodeState::kFree);
      }
  }
  ~Map() {}

  int Width() const { return width_; }
  int Height() const { return height_; }

  std::vector<NodeState> &operator[](int index) { return map_[index]; }

  void SetNodeState(const Node &node, NodeState node_state)
  {
    map_[node.X()][node.Y()] = node_state;
  }

private:
  int height_, width_;
  std::vector<std::vector<NodeState>> map_;
}; // class Map
} // namespace planning

#endif /* INCLUDE_UTILS_H_ */