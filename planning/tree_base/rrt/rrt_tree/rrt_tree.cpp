/**
 * @file rrt_tree.cpp
 * @author Yusuf Etkin KIZILDAĞ (etkin866@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/tree_base/rrt/rrt_tree/rrt_tree.h"

namespace planning
{
namespace tree_base
{

[[nodiscard]] auto RRTTree::getRoot() const noexcept -> std::shared_ptr<NodeParent<Cost>> { return root_; }

[[nodiscard]] auto RRTTree::insertNode(std::shared_ptr<Node> newNode, std::shared_ptr<NodeParent<Cost>> nearestNode,
                                       Cost const cost) -> bool
{
  auto const newNodeParent{std::make_shared<NodeParent<Cost>>(newNode, nearestNode, cost)};
  nodes_.push_back(newNodeParent);
  return true;
}

[[nodiscard]] auto RRTTree::getNodes() const noexcept -> std::vector<std::shared_ptr<NodeParent<Cost>>>
{
  return nodes_;
}

auto RRTTree::setRoot(const Node &root) noexcept -> void { root_ = std::make_shared<NodeParent<Cost>>(root); }

} // namespace tree_base
} // namespace planning
