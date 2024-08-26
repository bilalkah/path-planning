/**
 * @file tree_visualizer.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef TOOLS_INCLUDE_VISUALIZER_H_
#define TOOLS_INCLUDE_VISUALIZER_H_

#include "SDL3/SDL.h"
#include "utility/common_planning.h"
#include "utility/i_planning.h"
#include "utility/common_tree_base.h"
#include <cstddef>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>

namespace tools
{

using pair_size_t = std::pair<std::size_t, std::size_t>;
using pair_double = std::pair<double, double>;

inline std::unordered_map<planning::NodeState, SDL_Color> GetColorMap()
{
  std::unordered_map<planning::NodeState, SDL_Color> colors_;
  colors_.insert({planning::NodeState::kFree, SDL_Color{255, 255, 255, 255}});
  colors_.insert({planning::NodeState::kVisited, SDL_Color{0, 0, 255, 255}});
  colors_.insert({planning::NodeState::kOccupied, SDL_Color{0, 0, 0, 255}});
  colors_.insert({planning::NodeState::kStart, SDL_Color{0, 255, 0, 255}});
  colors_.insert({planning::NodeState::kGoal, SDL_Color{255, 0, 0, 255}});
  colors_.insert({planning::NodeState::kPath, SDL_Color{255, 0, 0, 255}});

  return colors_;
}

class Visualizer
{
public:
  Visualizer(std::shared_ptr<planning::Map> map, pair_double size_coeff,
             double kDelay, std::string window_name, std::string planner_name);
  ~Visualizer() = default;
  void SetStartAndGoal(const planning::Node &start_node,
                       const planning::Node &goal_node);

  void RenderMap();

  void VizGridLog();
  void VizTreeLog();

  void SetGetLogFunction(std::function<planning::Log()> get_log_function)
  {
    get_log_function_ = get_log_function;
  }

  void Run();
  bool IsRunning() { return is_running_; }
  void CheckEvent();
  void ClearScreen();
  void DrawLine(planning::Node start, planning::Node end);
  void DrawFilledRectangle(planning::Node start, planning::Node end);

private:
  std::shared_ptr<planning::Map> map;
  pair_double size_coeff_;
  double kDelay_;
  std::string window_name_;
  std::string planner_name_;
  SDL_bool loopShouldStop = SDL_FALSE;
  bool is_running_ = true;

  std::unordered_map<planning::NodeState, SDL_Color> colors_;
  SDL_Window *win = NULL;
  SDL_Renderer *renderer = NULL;
  std::function<void()> viz_function_;
  std::function<planning::Log()> get_log_function_;

  planning::Node start_node_;
  planning::Node goal_node_;
  std::thread window_thread_;
};

} // namespace tools

#endif // TOOLS_INCLUDE_VISUALIZER_H_
