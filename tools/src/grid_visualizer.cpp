/**
 * @file grid_visualizer.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tools/include/grid_visualizer.h"
#include "planning/include/common_planning.h"
#include <SFML/Graphics.hpp>

namespace tools
{

GridVisualizer::GridVisualizer(pair_size_t size, pair_double size_coeff,
                               std::size_t kDelay, std::string window_name)
    : window_name_(window_name), size_(size), size_coeff_(size_coeff),
      kDelay_(kDelay)
{
  window_.create(sf::VideoMode(size_.first * size_coeff_.first,
                               size_.second * size_coeff_.second),
                 window_name_);
  colors_ = GetColorMap();
};

GridVisualizer::~GridVisualizer() { window_.close(); }

void GridVisualizer::Visualize(const void *data)
{
  auto *log = static_cast<const planning::Map *>(data);

  window_.clear();

  for (std::size_t i = 0; i < log->GetHeight(); i++)
    {
      for (std::size_t j = 0; j < log->GetWidth(); j++)
        {
          sf::RectangleShape rectangle;
          rectangle.setSize(
              sf::Vector2f(size_coeff_.second, size_coeff_.first));
          rectangle.setPosition(j * size_coeff_.first, i * size_coeff_.second);
          rectangle.setFillColor(GetColorMap().at(
              log->GetNodeState({static_cast<int>(i), static_cast<int>(j)})));
          window_.draw(rectangle);
        }
    }

  // Draw start and goal nodes as circles
  window_.draw(start_node_);
  window_.draw(goal_node_);

  window_.display();
  sf::sleep(sf::milliseconds(kDelay_));
}

void GridVisualizer::SetStartAndGoal(const planning::Node &start_node,
                                     const planning::Node &goal_node)
{
  start_node_.setRadius(size_coeff_.first * 2);
  start_node_.setFillColor(GetColorMap().at(planning::NodeState::kStart));
  start_node_.setPosition(start_node.y_ * size_coeff_.second,
                          start_node.x_ * size_coeff_.first);

  goal_node_.setRadius(size_coeff_.first * 2);
  goal_node_.setFillColor(GetColorMap().at(planning::NodeState::kGoal));
  goal_node_.setPosition(goal_node.y_ * size_coeff_.second,
                         goal_node.x_ * size_coeff_.first);
}

} // namespace tools