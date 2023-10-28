/**
 * @file tree_visualizer.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tools/include/tree_visualizer.h"
#include "planning/include/data_types.h"
#include "planning/tree_base/include/common_tree_base.h"
#include <chrono>
#include <thread>
#include <vector>

namespace tools
{

TreeVisualizer::TreeVisualizer(std::shared_ptr<planning::Map> map,
                               pair_double size_coeff, std::size_t kDelay,
                               std::string window_name)
    : window_name_(window_name), map(map), size_coeff_(size_coeff),
      kDelay_(kDelay)
{
  window_.create(sf::VideoMode(map->GetHeight() * size_coeff_.first,
                               map->GetWidth() * size_coeff_.second),
                 window_name_);
  colors_ = GetColorMap();
  MapToTexture();

  window_thread_ = std::thread(&TreeVisualizer::Run, this);
  window_thread_.detach();
};

TreeVisualizer::~TreeVisualizer() { window_.close(); }

void TreeVisualizer::Visualize(
    const std::vector<std::shared_ptr<planning::NodeParent>> node_parent_vector,
    const std::shared_ptr<planning::NodeParent> goal_node_parent)
{

  {
    window_.clear();
    sf::Sprite sprite(render_texture_.getTexture());
    window_.draw(sprite);
  }
  for (auto &node_parent : node_parent_vector)
    {
      if (node_parent->parent == nullptr)
        {
          continue;
        }

      // sf line
      sf::Vertex line[] = {
          sf::Vertex(
              sf::Vector2f(node_parent->parent->node.y_ * size_coeff_.second,
                           node_parent->parent->node.x_ * size_coeff_.first),
              colors_.at(planning::NodeState::kVisited)),
          sf::Vertex(sf::Vector2f(node_parent->node.y_ * size_coeff_.second,
                                  node_parent->node.x_ * size_coeff_.first),
                     colors_.at(planning::NodeState::kVisited))};
      {
        window_.draw(line, 2, sf::Lines);
      }
    }

  if (goal_node_parent != nullptr)
    {
      auto path{planning::ReconstructPath(goal_node_parent)};

      // sf line
      for (auto i = 0u; i < path.size() - 1; i++)
        {
          sf::Vertex line[] = {
              sf::Vertex(sf::Vector2f(path[i].y_ * size_coeff_.second,
                                      path[i].x_ * size_coeff_.first),
                         colors_.at(planning::NodeState::kPath)),
              sf::Vertex(sf::Vector2f(path[i + 1].y_ * size_coeff_.second,
                                      path[i + 1].x_ * size_coeff_.first),
                         colors_.at(planning::NodeState::kPath))};

          {
            window_.draw(line, 2, sf::Lines);
          }
        }
    }

  // Draw start and goal nodes as circles
  {
    window_.draw(start_node_);
    window_.draw(goal_node_);
  }
  window_.display();
  // sleep scheduler for kDelay_ ms
  sf::sleep(sf::milliseconds(kDelay_));
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 120));
  sf::sleep(sf::milliseconds(1000 / 60));
}

void TreeVisualizer::SetStartAndGoal(const planning::Node &start_node,
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

void TreeVisualizer::MapToTexture()
{
  render_texture_.create(map->GetHeight() * size_coeff_.first,
                         map->GetWidth() * size_coeff_.second);
  render_texture_.clear();
  for (auto i = 0u; i < map->GetHeight(); i++)
    {
      for (auto j = 0u; j < map->GetWidth(); j++)
        {
          sf::RectangleShape rectangle;
          rectangle.setSize(
              sf::Vector2f(size_coeff_.second, size_coeff_.first));
          rectangle.setPosition(j * size_coeff_.second, i * size_coeff_.first);
          auto key = map->GetNodeState(planning::Node(i, j));
          rectangle.setFillColor(colors_.at(key));
          render_texture_.draw(rectangle);
        }
    }
  render_texture_.display();
}

void TreeVisualizer::Run()
{
  while (window_.isOpen())
    {
      sf::Event event;
      while (window_.pollEvent(event))
        {
          if (event.type == sf::Event::Closed)
            {
              window_.close();
            }
        }
    }
}

} // namespace tools
