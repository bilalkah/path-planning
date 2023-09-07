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

[[nodiscard]] auto RRTTree::insertNode(std::shared_ptr<Node> newNode, std::shared_ptr<Node> nearestNode)
    -> bool
{
  nodes_.emplace_back(std::make_shared<NodeParent<Cost>>(newNode, nearestNode));
  return true;
}

[[nodiscard]] auto RRTTree::getNodes() const noexcept -> std::vector<std::shared_ptr<NodeParent<Cost>>>
{
  return nodes_;
}

auto RRTTree::setRoot(const Node &root) noexcept -> void { root_ = std::make_shared<NodeParent<Cost>>(root, nullptr, Cost{}); }

[[nodiscard]] auto RRTTree::GetNearestNode(const Node &node) -> Node
{
  const auto min{
      *std::min_element(std::cbegin(nodes_), std::cend(nodes_),
                        [node](const std::shared_ptr<NodeParent<Cost>> &a, const std::shared_ptr<NodeParent<Cost>> &b) {
                          return GetDistance(node, *(a->node)) < GetDistance(node, *(b->node));
                        })};
  return *(min->node);
}

} // namespace tree_base
} // namespace planning
