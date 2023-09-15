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
#include "planning/include/i_planning.h"

#include <SFML/Graphics.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

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
   * @brief Writes map_ and updates window_. Thread safe.
   * @param map
   */
  void UpdateMap(const planning::Map &map);

  /**
   * @brief Visualize path. Thread safe.
   *
   * @param path
   */
  void VisualizeGridLog(const planning::Log &log);

  /**
   * @brief Visualize path. Thread safe.
   *
   * @param path
   */
  void VisualizeTreeLog(const planning::Log &log, const int delay);

  /**
   * @brief Visualize path. Thread safe.
   *
   * @param path
   */
  void VisualizeGridPath(const planning::Path &path);

  /**
   * @brief Visualize path. Thread safe.
   *
   * @param path
   */
  void VisualizeTreePath(const planning::Path &path);

  /**
   * @brief Writes to window_. Thread safe.
   *
   * @param node1
   * @param node2
   * @param state
   */
  void UpdateLine(const planning::Node &node1, const planning::Node &node2,
                  const planning::NodeState state);

  /**
   * @brief Update window_ with given position and state of node. Thread safe.
   * Calls SetColor()
   * @param node
   * @param state
   */
  void UpdateNode(const planning::Node &node, const planning::NodeState state);

private:
  /**
   * @brief Reads map_ and updates window_. NOT Thread safe.
   *  Calls SetColor()
   */
  void SetWindow();

  /**
   * @brief  Writes to window_. NOT Thread safe.
   *
   * @param node
   * @param color
   */
  void SetColor(const planning::Node &node, const planning::NodeState color);

  /**
   * @brief Reads from window_. Thread safe.
   *
   * @return int
   */
  int RenderWindow();

  /**
   * @brief Initialize colors
   *
   */
  void MapColor();

  sf::RenderWindow window_{};
  sf::RectangleShape cell_{};
  planning::Map map_;

  std::pair<float, float> cell_size_{}; // width, height
  int delay_{};
  bool show_{};

  std::thread draw_thread_{};
  std::mutex window_mutex_{};

  std::unordered_map<planning::NodeState, sf::Color> colors_ = {};

}; // class Visualizer

} // namespace tools

#endif // TOOLS_INCLUDE_VISUALIZER_H