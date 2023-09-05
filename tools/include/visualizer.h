/**
 * @file visualizer.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief 2D visualization
 * @version 0.1
 * @date 2023-09-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TOOLS_INCLUDE_VISUALIZER_H
#define TOOLS_INCLUDE_VISUALIZER_H

#include "planning/include/common_planning.h"
#include "planning/include/data_types.h"
#include <SFML/Graphics.hpp>

#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using std::chrono::milliseconds;

namespace tools
{

/**
 * @brief Visualizer class for 2D visualization
 *
 */
class Visualizer
{
public:
  Visualizer(planning::Map &map, float height_coeff, float width_coeff,
             int kDelay, bool kShow);

  ~Visualizer();

  /**
   * @brief Update whole window
   *
   * @param map
   */
  void UpdateMap(const planning::Map &map);

  /**
   * @brief Update cell with given position and state
   *
   * @param node
   * @param state
   */
  void UpdateCell(const planning::Node &node, const planning::NodeState state);

private:
  /**
   * @brief Initialize colors
   *
   */
  void MapColor();

  /**
   * @brief Thread function for drawing window
   *
   * @return int
   */
  int DrawWindow();

  /**
   * @brief Update window
   *
   */
  void UpdateScreen();

  /**
   * @brief Change color of cell
   *
   * @param node
   * @param color
   */
  void SetColor(const planning::Node &node, const planning::NodeState color);

  std::thread draw_thread_{};
  std::mutex window_mutex_{};

  sf::RenderWindow window_{};
  sf::RectangleShape cell_{};

  int delay_{};
  std::pair<float, float> cell_size_{};
  bool show_{};

  planning::Map map_;

  std::unordered_map<planning::NodeState, sf::Color> colors_ = {};

}; // class Visualizer

} // namespace tools

#endif // TOOLS_INCLUDE_VISUALIZER_H