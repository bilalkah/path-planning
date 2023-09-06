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
#include "planning/grid_base/dfs/dfs.h"
#include "planning/grid_base/include/common_grid_base.h"
#include "tools/include/visualizer.h"

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

  std::shared_ptr<planning::IPlanning> planner = std::make_shared<
      planning::grid_base::AStar<planning::grid_base::Directions4>>(
      planning::grid_base::four_directions);

  std::string dataDirectory = DATA_DIR;
  std::string dataFilePath = dataDirectory + "/bg2/AR0205SR.map";
  int kFactor = 2;
  const auto start_node = planning::Node(215, 25);
  const auto goal_node = planning::Node(330, 475);

  auto map = std::make_shared<planning::Map>(dataFilePath);
  tools::Visualizer visualizer(*map, kFactor, kFactor, 1, true);

  planning::Path path = planner->FindPath(start_node, goal_node, map);

  auto log = planner->GetLog();

  while (true)
    {
      for (const auto &node : log)
        {
          visualizer.UpdateNode(node.first, node.second);
        }

      // sleep for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      for (const auto &node : path)
        {
          visualizer.UpdateNode(*node, planning::NodeState::kPath);
          // sleep for 1 second
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

      // sleep for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      visualizer.UpdateMap(*map);

      // sleep for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  return 0;
}
