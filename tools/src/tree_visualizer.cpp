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
#include <vector>

namespace tools
{

TreeVisualizer::TreeVisualizer(pair_size_t size, pair_double size_coeff,
                               std::size_t kDelay, std::string window_name)
    : window_name_(window_name), size_(size), size_coeff_(size_coeff),
      kDelay_(kDelay)
{
  window_.create(sf::VideoMode(size_.first * size_coeff_.first,
                               size_.second * size_coeff_.second),
                 window_name_);
  colors_ = GetColorMap();
};

TreeVisualizer::~TreeVisualizer() { window_.close(); }

void TreeVisualizer::Visualize(const void *data)
{
  // data is std::vector<std::shared_ptr<planning::tree_based::Cost>>
  auto *node_parent_vector = static_cast<const std::vector<
      std::shared_ptr<planning::NodeParent<planning::tree_base::Cost>>> *>(
      data);

  window_.clear();

  for (auto &node_parent : *node_parent_vector)
    {
      if (node_parent->parent == nullptr)
        {
          continue;
        }

      auto ray{planning::tree_base::Get2DRayBetweenNodes(
          *node_parent->parent->node, *node_parent->node)};

      for (auto &node : ray)
        {
          sf::RectangleShape rectangle;
          rectangle.setSize(
              sf::Vector2f(size_coeff_.second, size_coeff_.first));
          rectangle.setPosition(node.y_ * size_coeff_.second,
                                node.x_ * size_coeff_.first);
          rectangle.setFillColor(
              GetColorMap().at(planning::NodeState::kVisited));
          window_.draw(rectangle);
        }
    }

  // Draw start and goal nodes as circles
  window_.draw(start_node_);
  window_.draw(goal_node_);

  window_.display();
  sf::sleep(sf::milliseconds(kDelay_));
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

} // namespace tools
