/**
 * @file visualizer.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tools/include/visualizer.h"
#include "planning/include/common_planning.h"
#include "planning/include/data_types.h"
#include <iostream>
#include <memory>
#include <mutex>
#include <sys/types.h>
#include <utility>

namespace tools
{

Visualizer::Visualizer(planning::Map &map, float height_coeff = 1.0f,
                       float width_coeff = 1.0f, int kDelay = 0,
                       bool kShow = true)
    : window_(sf::VideoMode(map.GetWidth() * width_coeff,
                            map.GetHeight() * height_coeff),
              "Path Planning Visualizer"),
      cell_(sf::Vector2f(1.0f * width_coeff, 1.0f * height_coeff)),
      delay_(kDelay), cell_size_(std::make_pair(width_coeff, height_coeff)),
      show_(kShow), map_(map)
{
  window_.setFramerateLimit(60);
  MapColor();
  UpdateMap(map);

  draw_thread_ = std::thread(&Visualizer::DrawWindow, this);
  draw_thread_.detach();

  std::cout << "Visualizer" << std::endl;
}

Visualizer::~Visualizer()
{
  show_ = false;
  draw_thread_.join();
  window_.close();

  std::cout << "~Visualizer" << std::endl;
}

void Visualizer::MapColor()
{
  colors_.insert({planning::NodeState::kFree, sf::Color::White});
  colors_.insert({planning::NodeState::kVisited, sf::Color::Blue});
  colors_.insert({planning::NodeState::kOccupied, sf::Color::Black});
  colors_.insert({planning::NodeState::kStart, sf::Color::Green});
  colors_.insert({planning::NodeState::kGoal, sf::Color::Red});
  colors_.insert({planning::NodeState::kPath, sf::Color::Yellow});

  std::cout << "MapColor" << std::endl;
}

void Visualizer::UpdateScreen()
{
  for (int i = 0; i < map_.GetWidth(); i++)
    {
      for (int j = 0; j < map_.GetHeight(); j++)
        {
          SetColor(planning::Node(i, j),
                   map_.GetNodeState(planning::Node(i, j)));
        }
    }
  std::cout << "UpdateWindow" << std::endl;
}

void Visualizer::UpdateCell(const planning::Node &node,
                            const planning::NodeState state)
{
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    SetColor(node, state);
  }
  std::cout << "UpdateWindow Cell" << std::endl;
}

void Visualizer::UpdateMap(const planning::Map &map)
{
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    map_ = map;
    UpdateScreen();
  }
  std::cout << "UpdateWindow Map" << std::endl;
}

int Visualizer::DrawWindow()
{
  while (window_.isOpen() && show_)
    {
      for (auto event = sf::Event{}; window_.pollEvent(event);)
        {
          if (event.type == sf::Event::Closed)
            {
              window_.close();
            }
        }

      {
        std::lock_guard<std::mutex> lock(window_mutex_);
        {
          window_.display();
        }
      }
      // Delay
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_));

      std::cout << "DrawWindow" << std::endl;
    }
  std::cout << "DrawWindow exit" << std::endl;
  return 0;
}

void Visualizer::SetColor(const planning::Node &node,
                          const planning::NodeState color)
{
  cell_.setPosition(node.x_ * cell_size_.first, node.y_ * cell_size_.second);
  cell_.setFillColor(colors_[color]);
  window_.draw(cell_);
}

} // namespace tools
