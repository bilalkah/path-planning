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

#include "planning/include/i_planning.h"
#include "planning/tree_base/rrt/rrt.h"
#include "planning/tree_base/rrt_star/rrt_star.h"
#include "tools/include/visualizer.h"
#include "yaml-cpp/yaml.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

std::shared_ptr<planning::IPlanningWithLogging> GetPlanner(YAML::Node node);

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
  auto planner_name = config["planner"]["name"].as<std::string>();
  std::shared_ptr<planning::IPlanningWithLogging> planner =
      GetPlanner(config["planner"]);

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

  while (true)
    {
      // Get time
      auto start_time{std::chrono::high_resolution_clock::now()};
      planning::Path path = planner->FindPath(start_node, goal_node, map);
      auto end_time{std::chrono::high_resolution_clock::now()};
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end_time - start_time)
                          .count();
      std::cout << "Planning Duration: " << duration << " ms" << std::endl;
      // dynamic cast to get log

      auto log = planner->GetLog();

      std::cout << "Log size: " << log.size() << std::endl;
      if (planner_name == "astar" || planner_name == "bfs" ||
          planner_name == "dfs")
        {
          visualizer.VisualizeGridLog(log);

          // sleep for 1 second
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));

          if (!path.empty())
            {
              visualizer.VisualizeGridPath(path);
            }
        }
      else if (planner_name == "rrt" || planner_name == "rrt_star")
        {
          if (planner_name == "rrt_star")
            {
              auto log_vector =
                  std::dynamic_pointer_cast<planning::tree_base::RRTStar>(
                      planner)
                      ->GetLogVector();
              for (auto &log : log_vector)
                {
                  visualizer.VisualizeTreeLog(log.first, 0);
                  // sleep for 1 second
                  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                  if (!log.second.empty())
                    {
                      visualizer.VisualizeTreePath(log.second);
                    }
                  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                  visualizer.UpdateMap(*map);
                }
            }
          else
            {
              visualizer.VisualizeTreeLog(log, 10);
            }

          // sleep for 1 second
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          if (!path.empty())
            {
              visualizer.VisualizeTreePath(path);
            }
        }

      // sleep for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      visualizer.UpdateMap(*map);

      // sleep for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::cout << "\n\n\n-----------\n\n\n" << std::endl;
    }

  return 0;
}

std::shared_ptr<planning::IPlanningWithLogging> GetPlanner(YAML::Node node)
{
  auto name = node["name"].as<std::string>();
  auto search_space = node["search_space"].as<int>();
  std::cout << name << std::endl;

  if (name == "astar")
    {
      auto heuristic = node["heuristic_weight"].as<double>();
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
  else if (name == "rrt")
    {
      return std::make_shared<planning::tree_base::RRT>();
    }
  else if (name == "rrt_star")
    {
      return std::make_shared<planning::tree_base::RRTStar>();
    }
  else
    {
      std::cerr << "Invalid planner" << std::endl;
      std::abort();
    }
}
