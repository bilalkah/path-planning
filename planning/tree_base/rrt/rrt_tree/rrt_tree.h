/**
 * @file rrt_tree.h
 * @author Yusuf Etkin KIZILDAĞ (etkin866@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PLANNING_TREE_BASE_RRT_RRT_TREE_H_
#define PLANNING_TREE_BASE_RRT_RRT_TREE_H_

#include "planning/include/data_types.h"
#include "planning/include/node_parent.h"
#include "planning/tree_base/include/common_tree_base.h"
#include "planning/tree_base/include/random_node_generator.h"
#include <algorithm>

namespace planning
{
namespace tree_base
{

class RRTTree
{
public:
  auto setRoot(const Node &) noexcept -> void;
  [[nodiscard]] auto getRoot() const noexcept -> std::shared_ptr<NodeParent<Cost>>;
  [[nodiscard]] auto GetNearestNode(const Node &) -> Node;
  [[nodiscard]] auto insertNode(std::shared_ptr<Node>, std::shared_ptr<Node>) -> bool;
  [[nodiscard]] auto getNodes() const noexcept -> std::vector<std::shared_ptr<NodeParent<Cost>>>;

private:
  std::shared_ptr<NodeParent<Cost>> root_{nullptr};
  std::vector<std::shared_ptr<NodeParent<Cost>>> nodes_;
};

} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RRT_RRT_TREE_H_ */
