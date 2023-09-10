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
#include "yaml-cpp/yaml.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

std::shared_ptr<planning::IPlanning> GetPlanner(YAML::Node node);

int main(int argc, char **argv)
{
  // Get config and data directories
  std::string configDirectory = CONFIG_DIR;
  std::string dataDirectory = DATA_DIR;

  std::string configFilePath = configDirectory + "/main.yaml";
  std::ifstream config_file(configFilePath);
  YAML::Node config = YAML::LoadFile(configFilePath);

  // Map config
  std::string mapFilePath = dataDirectory + config["map"].as<std::string>();
  auto map = std::make_shared<planning::Map>(mapFilePath);

  // Planner config
  std::shared_ptr<planning::IPlanning> planner = GetPlanner(config["planner"]);

  // Visualizer config
  auto rescale = config["visualizer"]["rescale"].as<float>();
  auto delay = config["visualizer"]["delay"].as<float>();
  auto show = config["visualizer"]["show"].as<bool>();

  tools::Visualizer visualizer(*map, rescale, rescale, delay, show);

  // Path config
  auto s = config["path"]["start"];
  auto start_node = planning::Node(s["x"].as<int>(), s["y"].as<int>());
  auto g = config["path"]["goal"];
  auto goal_node = planning::Node(g["x"].as<int>(), g["y"].as<int>());

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

std::shared_ptr<planning::IPlanning> GetPlanner(YAML::Node node)
{
  auto name = node["name"].as<std::string>();
  auto search_space = node["search_space"].as<int>();
  std::cout << name << std::endl;
  std::cout << search_space << std::endl;

  if (name == "astar")
    {
      auto heuristic = node["heuristic_weight"].as<double>();
      std::cout << heuristic << std::endl;
      return std::make_shared<planning::grid_base::AStar>(heuristic,
                                                          search_space);
    }
  else if (name == "bfs")
    {
      return std::make_shared<planning::grid_base::BFS>(search_space);
    }
  else if (name == "dfs")
    {
      return std::make_shared<planning::grid_base::DFS>(search_space);
    }
  else
    {
      std::cerr << "Invalid planner" << std::endl;
      std::abort();
    }
}
