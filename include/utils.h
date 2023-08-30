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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

class Node
{
public:
  Node() : x_(0), y_(0) {}
  Node(int x, int y) : x_(x), y_(y) {}
  ~Node() {}

  int X() const { return x_; }
  int Y() const { return y_; }

private:
  int x_, y_;
}; // class Node

enum class NodeState : uint8_t
{
  kFree,
  kVisited,
  kOccupied,
  kStart,
  kGoal,
  kPath
}; // enum class NodeState

class Map
{
public:
  Map() : width_(0), height_(0){};
  Map(int width, int height) : width_(width), height_(height)
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

private:
  int width_, height_;
  std::vector<std::vector<NodeState>> map_;
}; // class Map

#endif /* INCLUDE_UTILS_H_ */