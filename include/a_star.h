/**
 * @file a_star.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief A star path finding algorithm for 2D grid maps.
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_A_STAR_H_
#define INCLUDE_A_STAR_H_

#include "include/i_path_finding.h"
#include "include/utils.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <sys/types.h>
#include <utility>
#include <vector>

namespace planning
{

using SearchSpace = std::vector<std::pair<uint8_t, uint8_t>>;

class AStar : public IPathFinding
{
public:
  AStar(std::string direction);
  ~AStar();

  Path FindPath(const Node &start_node, const Node &goal_node,
                const std::shared_ptr<Map> map) override;

private:
  bool InBounds(const Node &node, std::shared_ptr<Map> map);
  bool IsFree(const Node &node, std::shared_ptr<Map> map);
  bool IsGoal(const Node &node, const Node &goal_node);

  Path ReconstructPath(std::shared_ptr<NodeInfo> current_node);

  SearchSpace search_space_;
}; // class AStar
} // namespace planning

#endif /* INCLUDE_A_STAR_H_ */