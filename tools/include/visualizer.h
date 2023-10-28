/**
 * @file tree_visualizer.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TOOLS_INCLUDE_VISUALIZER_H_
#define TOOLS_INCLUDE_VISUALIZER_H_

#include "planning/include/data_types.h"
#include "planning/include/i_planning.h"
#include "planning/tree_base/include/common_tree_base.h"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <cstddef>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>

namespace tools
{

using pair_size_t = std::pair<std::size_t, std::size_t>;
using pair_double = std::pair<double, double>;

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

class Visualizer
{
public:
  Visualizer(std::shared_ptr<planning::Map> map, pair_double size_coeff,
             std::size_t kDelay, std::string window_name,
             std::string planner_name);
  ~Visualizer() { window_.close(); }

  void SetStartAndGoal(const planning::Node &start_node,
                       const planning::Node &goal_node);

  void MapToTexture();

  void VizGridLog();
  void VizTreeLog();

  void SetGetLogFunction(std::function<planning::Log()> get_log_function)
  {
    get_log_function_ = get_log_function;
  }

  void Run();

private:
  std::shared_ptr<planning::Map> map;
  pair_double size_coeff_;
  std::size_t kDelay_;
  std::string window_name_;
  std::string planner_name_;

  std::unordered_map<planning::NodeState, sf::Color> colors_;
  sf::RenderWindow window_;
  sf::RenderTexture render_texture_;
  std::function<void()> viz_function_;
  std::function<planning::Log()> get_log_function_;

  sf::CircleShape start_node_;
  sf::CircleShape goal_node_;
  std::thread window_thread_;
};

} // namespace tools

#endif // TOOLS_INCLUDE_VISUALIZER_H_
