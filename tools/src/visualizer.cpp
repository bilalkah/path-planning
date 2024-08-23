/**
 * @file tree_visualizer.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tools/include/visualizer.h"
#include "SDL3/SDL_error.h"
#include "SDL3/SDL_events.h"
#include "SDL3/SDL_init.h"
#include "SDL3/SDL_rect.h"
#include "SDL3/SDL_render.h"
#include "SDL3/SDL_video.h"
#include "planning/include/data_types.h"
#include <cstddef>
#include <thread>
#include <vector>

namespace tools
{

Visualizer::Visualizer(std::shared_ptr<planning::Map> map,
                       pair_double size_coeff, double kDelay,
                       std::string window_name, std::string planner_name)
    : map(map), size_coeff_(size_coeff), kDelay_(kDelay),
      window_name_(window_name), planner_name_(planner_name)
{
  colors_ = GetColorMap();

  if (planner_name_ == "astar" || planner_name_ == "bfs" ||
      planner_name_ == "dfs")
    {
      viz_function_ = std::bind(&Visualizer::VizGridLog, this);
    }
  else if (planner_name_ == "rrt" || planner_name_ == "rrt_star")
    {
      viz_function_ = std::bind(&Visualizer::VizTreeLog, this);
    }
  else
    {
      throw std::runtime_error("Unknown planner name");
    }
  is_running_ = true;
  window_thread_ = std::thread([this]() { Run(); });
  window_thread_.detach();
};

void Visualizer::SetStartAndGoal(const planning::Node &start_node,
                                 const planning::Node &goal_node)
{
  start_node_ = start_node;
  goal_node_ = goal_node;
}

void Visualizer::RenderMap()
{

  for (auto i = 0u; i < map->GetWidth(); i++)
    {
      for (auto j = 0u; j < map->GetHeight(); j++)
        {

          auto key = map->GetNodeState(planning::Node(i, j));
          auto color = colors_.at(key);
          SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

          DrawFilledRectangle(
              {int(size_coeff_.first * j), int(i * size_coeff_.second)},
              {int(size_coeff_.first * (j + 1)),
               int(size_coeff_.second * (i + 1))});
        }
    }
}

void Visualizer::VizGridLog()
{
  if (get_log_function_ == nullptr)
    {
      return;
    }
  auto log = get_log_function_();
  auto color = colors_.at(planning::NodeState::kVisited);
  SDL_FRect *points = new SDL_FRect[log.first.size()];
  for (int i = 0; i < log.first.size(); i++)
    {
      points[i].x = log.first[i]->node.y_ * size_coeff_.second;
      points[i].y = log.first[i]->node.x_ * size_coeff_.first;
      points[i].w = size_coeff_.second;
      points[i].h = size_coeff_.first;
    }
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  SDL_RenderFillRects(renderer, points, log.first.size());
  delete[] points;

  if (log.second != nullptr)
    {
      auto path{planning::ReconstructPath(log.second)};
      SDL_FRect *points2 = new SDL_FRect[path.size()];

      for (int i = 0; i < path.size(); i++)
        {
          points2[i].x = path[i].y_ * size_coeff_.second;
          points2[i].y = path[i].x_ * size_coeff_.first;
          points2[i].w = size_coeff_.second;
          points2[i].h = size_coeff_.first;
        }
      color = colors_.at(planning::NodeState::kPath);
      SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
      SDL_RenderFillRects(renderer, points2, path.size());
    }
}

void Visualizer::VizTreeLog()
{
  if (get_log_function_ == nullptr)
    {
      return;
    }
  auto log = get_log_function_();
  auto color = colors_.at(planning::NodeState::kVisited);
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  for (auto &node_parent : log.first)
    {
      if (node_parent->parent == nullptr)
        {
          continue;
        }

      DrawLine(planning::Node(node_parent->parent->node.y_ * size_coeff_.second,
                              node_parent->parent->node.x_ * size_coeff_.first),
               planning::Node(node_parent->node.y_ * size_coeff_.second,
                              node_parent->node.x_ * size_coeff_.first));
    }

  if (log.second != nullptr)
    {
      auto path{planning::ReconstructPath(log.second)};
      color = colors_.at(planning::NodeState::kPath);
      SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

      for (auto i = 0u; i < path.size() - 1; i++)
        {
          DrawLine(planning::Node(path[i].y_ * size_coeff_.second,
                                  path[i].x_ * size_coeff_.first),
                   planning::Node(path[i + 1].y_ * size_coeff_.second,
                                  path[i + 1].x_ * size_coeff_.first));
        }
    }
}

void Visualizer::Run()
{
  if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
      std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
      exit(EXIT_FAILURE);
    }

  win = SDL_CreateWindow(window_name_.c_str(),
                         map->GetWidth() * size_coeff_.second,
                         map->GetHeight() * size_coeff_.first, 0);
  if (win == nullptr)
    {
      std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
      SDL_Quit();
      exit(EXIT_FAILURE);
    }

  renderer = SDL_CreateRenderer(win, NULL);
  if (renderer == nullptr)
    {
      std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
      SDL_DestroyWindow(win);
      SDL_Quit();
      exit(EXIT_FAILURE);
    }

  while (!loopShouldStop)
    {
      CheckEvent();
      ClearScreen();
      RenderMap();
      viz_function_();
      SDL_RenderPresent(renderer);
    }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(win);
  SDL_Quit();
}

void Visualizer::CheckEvent()
{
  SDL_Event event;
  while (SDL_PollEvent(&event))
    {
      // When user close the window
      if (event.type == SDL_EVENT_QUIT ||
          (event.type == SDL_EventType::SDL_EVENT_KEY_DOWN &&
           event.key.key == SDLK_ESCAPE))
        {
          loopShouldStop = SDL_TRUE;
          is_running_ = false;
        }
    }
}

void Visualizer::ClearScreen()
{
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
  SDL_RenderClear(renderer);
}

void Visualizer::DrawLine(planning::Node start, planning::Node end)
{
  SDL_RenderLine(renderer, start.x_, start.y_, end.x_, end.y_);
}

void Visualizer::DrawFilledRectangle(planning::Node start, planning::Node end)
{
  SDL_FRect rect{float(start.x_), float(start.y_), float(end.x_ - start.x_),
                 float(end.y_ - start.y_)};
  SDL_RenderFillRect(renderer, &rect);
}

} // namespace tools
