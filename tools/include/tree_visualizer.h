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

#ifndef TOOLS_INCLUDE_TREE_VISUALIZER_H_
#define TOOLS_INCLUDE_TREE_VISUALIZER_H_

#include "planning/tree_base/include/common_tree_base.h"
#include "tools/include/i_visualize.h"
#include <SFML/Graphics.hpp>
#include <memory>

namespace tools
{

class TreeVisualizer
{
public:
  TreeVisualizer(std::shared_ptr<planning::Map> map, pair_double size_coeff,
                 std::size_t kDelay, std::string window_name);
  ~TreeVisualizer();

  void Visualize(const std::vector<std::shared_ptr<planning::NodeParent>>
                     node_parent_vector,
                 const std::shared_ptr<planning::NodeParent> goal_node_parent);

  void SetStartAndGoal(const planning::Node &start_node,
                       const planning::Node &goal_node);

private:
  sf::RenderWindow window_;
  std::string window_name_;
  std::shared_ptr<planning::Map> map;
  pair_double size_coeff_;
  std::size_t kDelay_;

  std::unordered_map<planning::NodeState, sf::Color> colors_;
  sf::CircleShape start_node_;
  sf::CircleShape goal_node_;
};

} // namespace tools

#endif // TOOLS_INCLUDE_TREE_VISUALIZER_H_
