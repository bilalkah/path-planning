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

RRTTree::RRTTree(std::shared_ptr<Node> root) { root_ = std::make_shared<NodeParent<Cost>>(root, nullptr, Cost{}); }

[[nodiscard]] auto RRTTree::getRoot() const noexcept -> std::shared_ptr<NodeParent<Cost>> { return root_; }

auto RRTTree::insertNode(std::shared_ptr<Node> newNode, std::shared_ptr<NodeParent<Cost>> nearestNode, Cost const cost)
    -> void
{
  auto const newNodeParent{std::make_shared<NodeParent<Cost>>(newNode, nearestNode, cost)};
  nodes_.push_back(newNodeParent);
}

[[nodiscard]] auto RRTTree::getNodes() const noexcept -> std::vector<std::shared_ptr<NodeParent<Cost>>>
{
  return nodes_;
}

} // namespace tree_base
} // namespace planning
