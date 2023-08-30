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

#include <cstdint>
#include <memory>
#include <vector>

class Node
{
public:
  Node();
  ~Node();

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
  Map();
  ~Map();
  int Width() const { return width_; }
  int Height() const { return height_; }

  std::vector<NodeState> &operator[](int index) { return map_[index]; }

private:
  int width_, height_;
  std::vector<std::vector<NodeState>> map_;
}; // class Map

#endif /* INCLUDE_UTILS_H_ */