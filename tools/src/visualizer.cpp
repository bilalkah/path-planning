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

#include "tools/include/visualizer.h"
#include "planning/include/data_types.h"
#include "planning/tree_base/include/common_tree_base.h"
#include <chrono>
#include <thread>
#include <vector>

namespace tools
{

Visualizer::Visualizer(std::shared_ptr<planning::Map> map,
                       pair_double size_coeff, std::size_t kDelay,
                       std::string window_name, std::string planner_name)
    : map(map), size_coeff_(size_coeff), kDelay_(kDelay),
      window_name_(window_name), planner_name_(planner_name)
{
  colors_ = GetColorMap();
  window_.create(sf::VideoMode(map->GetHeight() * size_coeff_.first,
                               map->GetWidth() * size_coeff_.second),
                 window_name_);
  MapToTexture();

  if (planner_name_ == "astar" || planner_name_ == "bfs" ||
      planner_name_ == "dfs")
    {
      viz_function_ = std::bind(&Visualizer::VizGridLog, this);
    }
  else if (planner_name_ == "rrt" || planner_name_ == "rrt_star")
    {
      viz_function_ = std::bind(&Visualizer::VizTreeLog, this);
    }
  else
    {
      throw std::runtime_error("Unknown planner name");
    }

  window_thread_ = std::thread(&Visualizer::Run, this);
  window_thread_.detach();
};

void Visualizer::SetStartAndGoal(const planning::Node &start_node,
                                 const planning::Node &goal_node)
{
  start_node_.setRadius(size_coeff_.first * 2);
  start_node_.setFillColor(colors_.at(planning::NodeState::kStart));
  start_node_.setPosition(start_node.y_ * size_coeff_.second,
                          start_node.x_ * size_coeff_.first);

  goal_node_.setRadius(size_coeff_.first * 2);
  goal_node_.setFillColor(colors_.at(planning::NodeState::kGoal));
  goal_node_.setPosition(goal_node.y_ * size_coeff_.second,
                         goal_node.x_ * size_coeff_.first);
}

void Visualizer::MapToTexture()
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

void Visualizer::VizGridLog()
{
  if (get_log_function_ == nullptr)
    {
      return;
    }
  auto log = get_log_function_();
  sf::RectangleShape cell_{};
  cell_.setSize(sf::Vector2f(size_coeff_.second, size_coeff_.first));
  for (auto &node_parent : log.first)
    {
      if (node_parent == nullptr)
        {
          continue;
        }
      cell_.setPosition(node_parent->node.y_ * size_coeff_.second,
                        node_parent->node.x_ * size_coeff_.first);
      cell_.setFillColor(colors_.at(planning::NodeState::kVisited));
      window_.draw(cell_);
    }

  if (log.second != nullptr)
    {
      auto path{planning::ReconstructPath(log.second)};
      for (auto &node : path)
        {
          cell_.setPosition(node.y_ * size_coeff_.second,
                            node.x_ * size_coeff_.first);
          cell_.setFillColor(colors_.at(planning::NodeState::kPath));
          window_.draw(cell_);
        }
    }
}

void Visualizer::VizTreeLog()
{
  if (get_log_function_ == nullptr)
    {
      return;
    }
  auto log = get_log_function_();
  for (auto &node_parent : log.first)
    {
      if (node_parent->parent == nullptr)
        {
          continue;
        }

      sf::Vertex line[] = {
          sf::Vertex(
              sf::Vector2f(node_parent->parent->node.y_ * size_coeff_.second,
                           node_parent->parent->node.x_ * size_coeff_.first),
              colors_.at(planning::NodeState::kVisited)),
          sf::Vertex(sf::Vector2f(node_parent->node.y_ * size_coeff_.second,
                                  node_parent->node.x_ * size_coeff_.first),
                     colors_.at(planning::NodeState::kVisited))};
      window_.draw(line, 2, sf::Lines);
    }

  if (log.second != nullptr)
    {
      auto path{planning::ReconstructPath(log.second)};

      for (auto i = 0u; i < path.size() - 1; i++)
        {
          sf::Vertex line[] = {
              sf::Vertex(sf::Vector2f(path[i].y_ * size_coeff_.second,
                                      path[i].x_ * size_coeff_.first),
                         colors_.at(planning::NodeState::kPath)),
              sf::Vertex(sf::Vector2f(path[i + 1].y_ * size_coeff_.second,
                                      path[i + 1].x_ * size_coeff_.first),
                         colors_.at(planning::NodeState::kPath))};
          window_.draw(line, 2, sf::Lines);
        }
    }
}

void Visualizer::Run()
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

      window_.clear();
      sf::Sprite sprite(render_texture_.getTexture());
      window_.draw(sprite);
      viz_function_();
      window_.draw(start_node_);
      window_.draw(goal_node_);
      window_.display();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 30));
    }
}

} // namespace tools
