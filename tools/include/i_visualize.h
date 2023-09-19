/**
 * @file i_visualize.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TOOLS_INCLUDE_I_VISUALIZE_H_
#define TOOLS_INCLUDE_I_VISUALIZE_H_

#include "planning/include/data_types.h"
#include <SFML/Graphics.hpp>
#include <cstddef>
#include <unordered_map>

namespace tools
{

using pair_size_t = std::pair<std::size_t, std::size_t>;
using pair_double = std::pair<double, double>;

class IVisualize
{
public:
  virtual void Visualize(const void *data) = 0;
  virtual void SetStartAndGoal(const planning::Node &start_node,
                               const planning::Node &goal_node) = 0;
  virtual ~IVisualize() {}
}; // class IVisualize

inline std::unordered_map<planning::NodeState, sf::Color> GetColorMap()
{
  std::unordered_map<planning::NodeState, sf::Color> colors_;
  colors_.insert({planning::NodeState::kFree, sf::Color::White});
  colors_.insert({planning::NodeState::kVisited, sf::Color::Blue});
  colors_.insert({planning::NodeState::kOccupied, sf::Color::Black});
  colors_.insert({planning::NodeState::kStart, sf::Color::Green});
  colors_.insert({planning::NodeState::kGoal, sf::Color::Yellow});
  colors_.insert({planning::NodeState::kPath, sf::Color::Red});

  return colors_;
}

} // namespace tools

#endif // TOOLS_INCLUDE_I_VISUALIZE_H_