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

#include "grid_base/astar/astar.h"
#include "grid_base/bfs/bfs.h"
#include "grid_base/dfs/dfs.h"
#include "utility/common_grid_base.h"

#include "tools/visualizer/visualizer.h"
#include "tree_base/rrt/rrt.h"
#include "tree_base/rrt_star/rrt_star.h"
#include "utility/i_planning.h"
#include "yaml-cpp/yaml.h"

#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

using PlannerType = std::shared_ptr<planning::IPlanningWithLogging>;

PlannerType GetGridBasedPlanner(std::string planner_name);
PlannerType GetTreeBasedPlanner(std::string planner_name);
PlannerType GetPlanner(std::string planner_name);

int main(int argc, char **argv)
{
  // Get config and data directories
  std::string config_directory = CONFIG_DIR;
  std::string data_directory = DATA_DIR;
  std::string config_file = config_directory + "/main.yaml";
  YAML::Node config = YAML::LoadFile(config_file);

  // Map config
  std::string map_file = data_directory + config["map"].as<std::string>();
  const auto map = std::make_shared<planning::Map>(map_file);

  // Planner config
  auto planner_name = config["planner_name"].as<std::string>();
  PlannerType planner = GetPlanner(planner_name);

  // Visualizer config
  auto rescale_factor = config["visualizer"]["rescale"].as<double>();
  auto delay = config["visualizer"]["delay"].as<double>();

  auto visualizer = std::make_shared<tools::Visualizer>(
      map, tools::pair_double{rescale_factor, rescale_factor}, delay,
      "Tree Visualizer", planner_name);

  // Path config
  auto s = config["path"]["start"];
  auto start_node = planning::Node(s["x"].as<int>(), s["y"].as<int>());
  auto g = config["path"]["goal"];
  auto goal_node = planning::Node(g["x"].as<int>(), g["y"].as<int>());

  // Visualize start and goal nodes
  visualizer->SetStartAndGoal(start_node, goal_node);

  while (visualizer->IsRunning())
    {
      visualizer->SetGetLogFunction(
          std::bind(&planning::IPlanningWithLogging::GetLog, planner));
      std::cout << "Started" << std::endl;
      // Get time
      auto start_time{std::chrono::high_resolution_clock::now()};
      planning::Path path = planner->FindPath(start_node, goal_node, map);
      auto end_time{std::chrono::high_resolution_clock::now()};
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end_time - start_time)
                          .count();
      std::cout << "Planning Duration: " << duration << " ms" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::cout << "Finished" << std::endl;
      // set null to get_log_function_ to stop logging
      visualizer->SetGetLogFunction(nullptr);
      planner->ClearLog();
    }

  return 0;
}

PlannerType GetPlanner(std::string planner_name)
{
  PlannerType result{};
  if (planner_name == "astar" || planner_name == "bfs" || planner_name == "dfs")
    {
      result = GetGridBasedPlanner(planner_name);
    }
  else if (planner_name == "rrt" || planner_name == "rrt_star")
    {
      result = GetTreeBasedPlanner(planner_name);
    }
  else
    {
      std::cout << "Invalid planner name" << std::endl;
      exit(1);
    }
  return result;
}

PlannerType GetGridBasedPlanner(std::string planner_name)
{
  std::string config_directory = CONFIG_DIR;
  std::string config_file = config_directory + "/grid_base.yaml";
  YAML::Node config = YAML::LoadFile(config_file);

  auto heuristic_weight = config["heuristic_weight"].as<double>();
  auto search_space = config["search_space"].as<int>();

  PlannerType planner;
  if (planner_name == "astar")
    {
      planner = std::make_shared<planning::grid_base::AStar>(heuristic_weight,
                                                             search_space);
    }
  else if (planner_name == "bfs")
    {
      planner = std::make_shared<planning::grid_base::BFS>(search_space);
    }
  else if (planner_name == "dfs")
    {
      planner = std::make_shared<planning::grid_base::DFS>(search_space);
    }
  else
    {
      std::cout << "Invalid planner name" << std::endl;
      exit(1);
    }
  return planner;
}
PlannerType GetTreeBasedPlanner(std::string planner_name)
{
  std::string config_directory = CONFIG_DIR;
  std::string config_file = config_directory + "/tree_base.yaml";
  YAML::Node config = YAML::LoadFile(config_file);

  auto max_iteration = config["max_iteration"].as<int>();
  auto max_branch_length = config["max_branch_length"].as<int>();
  auto min_branch_length = config["min_branch_length"].as<int>();
  auto neighbor_radius = config["neighbor_radius"].as<int>();
  auto goal_radius = config["goal_radius"].as<int>();

  PlannerType planner;
  if (planner_name == "rrt")
    {
      planner = std::make_shared<planning::tree_base::RRT>(
          max_iteration, max_branch_length, min_branch_length, goal_radius);
    }
  else if (planner_name == "rrt_star")
    {
      planner = std::make_shared<planning::tree_base::RRTStar>(
          max_iteration, max_branch_length, min_branch_length, neighbor_radius,
          goal_radius);
    }
  else
    {
      std::cout << "Invalid planner name" << std::endl;
      exit(1);
    }
  return planner;
}
