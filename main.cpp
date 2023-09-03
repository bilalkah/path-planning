/**
 * @file main.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "planning/grid_base/a_star/a_star.h"
#include "planning/grid_base/bfs/bfs.h"
#include "planning/grid_base/common_grid_base.h"
#include "planning/grid_base/dfs/dfs.h"
#include "planning/i_planning.h"

#include <iostream>
#include <memory>

int main(int argc, char **argv)
{
  // Arguments parser
  // --map_path <path_to_map>
  // --algorithm <planner_name>
  // --start <start_x> <start_y>
  // --goal <goal_x> <goal_y>
  // --output <output_path (gif file)>

  std::shared_ptr<planning::IPlanning> planner =
      std::make_shared<planning::grid_base::AStar>();

  std::cout << "main " << std::endl;

  return 0;
}