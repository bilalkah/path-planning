/**
 * @file planner_bindings.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief WebAssembly entry points for the project page.
 * @version 0.1
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 * Exposes the planners in planning/ to JavaScript through Embind. The page
 * writes a .map file into Emscripten's in-memory filesystem and
 * planning::Map reads it, as it does in main.cpp. Planners are built with
 * the same constructors main.cpp uses. Each run returns the path FindPath
 * returns and the planner's log, which is what tools/visualizer draws.
 */

#include "grid_base/astar/astar.h"
#include "grid_base/bfs/bfs.h"
#include "grid_base/dfs/dfs.h"
#include "tree_base/rrt/rrt.h"
#include "tree_base/rrt_star/rrt_star.h"
#include "utility/common_planning.h"
#include "utility/i_planning.h"

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

using emscripten::val;
using PlannerType = std::shared_ptr<planning::IPlanningWithLogging>;

// Maps read so far, keyed by their path in the in-memory filesystem.
std::unordered_map<std::string, std::shared_ptr<planning::Map>> maps;

// Copy into a new JavaScript typed array, so the result stays valid after
// the module's memory grows.
template <typename T>
val ToTypedArray(const char *type, const std::vector<T> &data)
{
  return val::global(type).new_(
      emscripten::typed_memory_view(data.size(), data.data()));
}

val Error(const std::string &message)
{
  val result = val::object();
  result.set("error", message);
  return result;
}

// Same planners and constructor arguments as GetGridBasedPlanner and
// GetTreeBasedPlanner in main.cpp. The parameters use the key names from
// config/grid_base.yaml and config/tree_base.yaml.
PlannerType MakePlanner(const std::string &planner_name, const val &params)
{
  if (planner_name == "astar")
    {
      return std::make_shared<planning::grid_base::AStar>(
          params["heuristic_weight"].as<double>(),
          params["search_space"].as<int>());
    }
  if (planner_name == "bfs")
    {
      return std::make_shared<planning::grid_base::BFS>(
          params["search_space"].as<int>());
    }
  if (planner_name == "dfs")
    {
      return std::make_shared<planning::grid_base::DFS>(
          params["search_space"].as<int>());
    }
  if (planner_name == "rrt")
    {
      return std::make_shared<planning::tree_base::RRT>(
          params["max_iteration"].as<int>(),
          params["max_branch_length"].as<int>(),
          params["min_branch_length"].as<int>(),
          params["goal_radius"].as<int>());
    }
  if (planner_name == "rrt_star")
    {
      return std::make_shared<planning::tree_base::RRTStar>(
          params["max_iteration"].as<int>(),
          params["max_branch_length"].as<int>(),
          params["min_branch_length"].as<int>(),
          params["neighbor_radius"].as<int>(),
          params["goal_radius"].as<int>());
    }
  return nullptr;
}

// Dropping the last reference to a NodeParent frees its parent chain
// recursively. A DFS chain can be tens of thousands of nodes deep, which is
// more than a browser's call stack allows. While the log still holds every
// node, cutting the parent links lets each node be freed on its own.
void UnlinkLog(planning::Log &log)
{
  for (auto &entry : log.first)
    {
      entry->parent.reset();
    }
  if (log.second != nullptr)
    {
      log.second->parent.reset();
    }
}

/**
 * @brief Write a .map file to the in-memory filesystem and read it with
 * planning::Map.
 *
 * @param path File name in the in-memory filesystem, such as
 * "/AR0205SR.map".
 * @param contents Text of the Moving AI .map file.
 * @return {height, width, cells}. cells holds one planning::NodeState per
 * cell, row by row: index x * width + y.
 */
val LoadMap(const std::string &path, const std::string &contents)
{
  if (contents.rfind("type", 0) != 0)
    {
      return Error("Not a Moving AI .map file: " + path);
    }
  {
    std::ofstream file(path, std::ios::binary);
    file << contents;
  }

  std::string file_path{path};
  auto map = std::make_shared<planning::Map>(file_path);
  maps[path] = map;

  const auto height = map->GetHeight();
  const auto width = map->GetWidth();
  std::vector<uint8_t> cells(height * width);
  for (auto x = 0u; x < height; x++)
    {
      for (auto y = 0u; y < width; y++)
        {
          cells[x * width + y] = static_cast<uint8_t>(map->GetNodeState(
              planning::Node(static_cast<int>(x), static_cast<int>(y))));
        }
    }

  val result = val::object();
  result.set("height", static_cast<double>(height));
  result.set("width", static_cast<double>(width));
  result.set("cells", ToTypedArray("Uint8Array", cells));
  return result;
}

/**
 * @brief Run one planner, as main.cpp does for each run.
 *
 * @return {found, timeMs, path, log}. path holds the (x, y) pairs FindPath
 * returned. log holds (x, y, parent) triples for each NodeParent in
 * GetLog().first, in the order the planner logged them. parent is the index
 * of the parent's triple, -1 for a root, and -2 for a parent the log does
 * not hold.
 */
val Plan(const std::string &map_path, const std::string &planner_name,
         int start_x, int start_y, int goal_x, int goal_y, const val &params)
{
  const auto found_map = maps.find(map_path);
  if (found_map == maps.end())
    {
      return Error("Map is not loaded: " + map_path);
    }
  const auto map = found_map->second;

  const planning::Node start_node(start_x, start_y);
  const planning::Node goal_node(goal_x, goal_y);
  if (!planning::IsFree(start_node, map) || !planning::IsFree(goal_node, map))
    {
      return Error("Start and goal must be free cells inside the map.");
    }

  PlannerType planner = MakePlanner(planner_name, params);
  if (planner == nullptr)
    {
      return Error("Unknown planner: " + planner_name);
    }

  const auto start_time{std::chrono::steady_clock::now()};
  const planning::Path path = planner->FindPath(start_node, goal_node, map);
  const auto end_time{std::chrono::steady_clock::now()};

  planning::Log log = planner->GetLog();

  std::unordered_map<const planning::NodeParent *, int32_t> index;
  index.reserve(log.first.size());
  for (auto i = 0u; i < log.first.size(); i++)
    {
      index.emplace(log.first[i].get(), static_cast<int32_t>(i));
    }

  std::vector<int32_t> log_nodes;
  log_nodes.reserve(log.first.size() * 3);
  for (const auto &entry : log.first)
    {
      int32_t parent = -1;
      if (entry->parent != nullptr)
        {
          const auto found_parent = index.find(entry->parent.get());
          parent = found_parent == index.end() ? -2 : found_parent->second;
        }
      log_nodes.push_back(entry->node.x_);
      log_nodes.push_back(entry->node.y_);
      log_nodes.push_back(parent);
    }

  std::vector<int32_t> path_nodes;
  path_nodes.reserve(path.size() * 2);
  for (const auto &node : path)
    {
      path_nodes.push_back(node.x_);
      path_nodes.push_back(node.y_);
    }

  UnlinkLog(log);

  val result = val::object();
  result.set("found", !path.empty());
  result.set("timeMs",
             std::chrono::duration<double, std::milli>(end_time - start_time)
                 .count());
  result.set("path", ToTypedArray("Int32Array", path_nodes));
  result.set("log", ToTypedArray("Int32Array", log_nodes));
  return result;
}

} // namespace

EMSCRIPTEN_BINDINGS(path_planning)
{
  emscripten::function("loadMap", &LoadMap);
  emscripten::function("plan", &Plan);
}
