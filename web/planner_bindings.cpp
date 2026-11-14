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
#include <wasi/api.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <streambuf>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using emscripten::val;
using PlannerType = std::shared_ptr<planning::IPlanningWithLogging>;

// Maps read so far, keyed by their path in the in-memory filesystem.
std::unordered_map<std::string, std::shared_ptr<planning::Map>> maps;

// Collects what the planners write to std::cout, line by line, and hands
// each line to a callback as it is written.
class LineCapture : public std::streambuf
{
public:
  explicit LineCapture(std::function<void(const std::string &)> on_line)
      : on_line_(std::move(on_line))
  {
  }

  const std::vector<std::string> &Lines() const { return lines_; }

  void Flush()
  {
    if (!current_.empty())
      {
        EndLine();
      }
  }

protected:
  int_type overflow(int_type ch) override
  {
    if (traits_type::eq_int_type(ch, traits_type::eof()))
      {
        return traits_type::not_eof(ch);
      }
    if (traits_type::to_char_type(ch) == '\n')
      {
        EndLine();
      }
    else
      {
        current_.push_back(traits_type::to_char_type(ch));
      }
    return ch;
  }

  std::streamsize xsputn(const char *text, std::streamsize count) override
  {
    for (std::streamsize i = 0; i < count; i++)
      {
        overflow(traits_type::to_int_type(text[i]));
      }
    return count;
  }

private:
  void EndLine()
  {
    lines_.push_back(current_);
    on_line_(current_);
    current_.clear();
  }

  std::function<void(const std::string &)> on_line_;
  std::vector<std::string> lines_{};
  std::string current_{};
};

// Points std::cout at another buffer for the lifetime of the object.
class RedirectCout
{
public:
  explicit RedirectCout(std::streambuf *buffer)
      : previous_(std::cout.rdbuf(buffer))
  {
  }
  ~RedirectCout() { std::cout.rdbuf(previous_); }

private:
  std::streambuf *previous_;
};

// Records the best path of an RRT* run each time it changes, with the number
// of logged nodes at that moment.
//
// RRT* keeps improving its path after it finds the first one, but its log
// holds only the final tree. The recorder looks at the planner between
// iterations instead: every iteration starts by drawing a random sample, and
// std::random_device reads it through getentropy (defined below) while the
// planner holds no lock. CheckIfGoalReached prints "Path found." or "Path
// changed." when a new goal node becomes the best one, and that node is then
// the last one in the log. Rewiring can also shorten the path without a new
// goal node, so the recorder compares the whole parent chain each time.
class PathRecorder
{
public:
  void Start(planning::IPlanningWithLogging *planner)
  {
    planner_ = planner;
    best_.reset();
    new_best_ = false;
    last_signature_ = 0;
    snapshots_.clear();
  }

  void Finish()
  {
    Sample();
    planner_ = nullptr;
    best_.reset();
  }

  void OnLine(const std::string &line)
  {
    if (line.rfind("Path found.", 0) == 0 ||
        line.rfind("Path changed.", 0) == 0)
      {
        new_best_ = true;
      }
  }

  void Sample()
  {
    if (planner_ == nullptr)
      {
        return;
      }
    if (new_best_)
      {
        new_best_ = false;
        best_ = planner_->GetLog().first.back();
      }
    if (best_ == nullptr)
      {
        return;
      }

    bool complete{false};
    const auto signature = Signature(best_, complete);
    if (!complete || signature == last_signature_)
      {
        return;
      }
    last_signature_ = signature;

    const planning::Path path = planning::ReconstructPath(best_);
    snapshots_.push_back(static_cast<int32_t>(planner_->GetLog().first.size()));
    snapshots_.push_back(static_cast<int32_t>(path.size()));
    for (const auto &node : path)
      {
        snapshots_.push_back(node.x_);
        snapshots_.push_back(node.y_);
      }
  }

  // Flattened records: logged nodes, point count, then (x, y) per point.
  const std::vector<int32_t> &Snapshots() const { return snapshots_; }

private:
  // A hash of the parent chain from the goal back to the root. `complete` is
  // false if the walk did not reach the root within the step limit.
  static uint64_t Signature(const std::shared_ptr<planning::NodeParent> &node,
                            bool &complete)
  {
    constexpr std::size_t kMaxSteps{1u << 20};
    uint64_t hash{14695981039346656037ull};
    std::size_t steps{0};
    const planning::NodeParent *current = node.get();
    while (current != nullptr && steps < kMaxSteps)
      {
        hash = (hash ^ static_cast<uint32_t>(current->node.x_)) *
               1099511628211ull;
        hash = (hash ^ static_cast<uint32_t>(current->node.y_)) *
               1099511628211ull;
        current = current->parent.get();
        steps++;
      }
    complete = current == nullptr;
    return hash;
  }

  planning::IPlanningWithLogging *planner_{nullptr};
  std::shared_ptr<planning::NodeParent> best_{};
  bool new_best_{false};
  uint64_t last_signature_{0};
  std::vector<int32_t> snapshots_{};
};

PathRecorder recorder;

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
 * @return {found, timeMs, path, log, progress, stdout}. path holds the (x, y)
 * pairs FindPath returned. log holds (x, y, parent) triples for each
 * NodeParent in GetLog().first, in the order the planner logged them. parent
 * is the index of the parent's triple, -1 for a root, and -2 for a parent the
 * log does not hold. progress holds the best path each time it changed during
 * an RRT* run (see PathRecorder). stdout holds the lines the planner printed.
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

  LineCapture capture(
      [](const std::string &line) { recorder.OnLine(line); });
  RedirectCout redirect(&capture);

  PlannerType planner = MakePlanner(planner_name, params);
  if (planner == nullptr)
    {
      return Error("Unknown planner: " + planner_name);
    }

  recorder.Start(planner.get());
  const auto start_time{std::chrono::steady_clock::now()};
  const planning::Path path = planner->FindPath(start_node, goal_node, map);
  const auto end_time{std::chrono::steady_clock::now()};
  recorder.Finish();
  capture.Flush();

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
  result.set("progress", ToTypedArray("Int32Array", recorder.Snapshots()));

  val lines = val::array();
  for (const auto &line : capture.Lines())
    {
      lines.call<void>("push", line);
    }
  result.set("stdout", lines);
  return result;
}

} // namespace

// std::random_device reads its numbers through getentropy. This definition
// replaces musl's, takes the bytes from the same source (__wasi_random_get),
// and first lets the path recorder look at the planner: RRT and RRT* draw a
// sample at the start of every iteration, while they hold no lock.
extern "C" int getentropy(void *buffer, size_t length)
{
  recorder.Sample();
  if (length > 256)
    {
      errno = EIO;
      return -1;
    }
  const auto status =
      __wasi_random_get(static_cast<uint8_t *>(buffer), length);
  if (status != __WASI_ERRNO_SUCCESS)
    {
      errno = status;
      return -1;
    }
  return 0;
}

EMSCRIPTEN_BINDINGS(path_planning)
{
  emscripten::function("loadMap", &LoadMap);
  emscripten::function("plan", &Plan);
}
