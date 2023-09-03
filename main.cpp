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
#include "planning/include/common_planning.h"
#include "planning/include/i_planning.h"

#include <iostream>
#include <memory>

#include <SFML/Graphics.hpp>

void drawMatrix(sf::RenderWindow &window,
                const std::shared_ptr<planning::Map> &map)
{
  const int numRows = map->GetHeight();
  const int numCols = map->GetWidth();
  const float cellWidth = 1.0f;
  const float cellHeight = 1.0f;

  for (int i = 0; i < numRows; ++i)
    {
      for (int j = 0; j < numCols; ++j)
        {
          sf::RectangleShape cell(sf::Vector2f(cellWidth, cellHeight));
          cell.setPosition(
              j * cellWidth,
              i * cellHeight); // Adjust position based on row and column
          if (map->GetNodeState(planning::Node(i, j)) ==
              planning::NodeState::kFree)
            {
              cell.setFillColor(sf::Color::White); // Modify color as needed
            }
          else
            {
              cell.setFillColor(sf::Color::Black); // Modify color as needed
            }
          window.draw(cell);
        }
    }
}

int main(int argc, char **argv)
{
  // Arguments parser
  // --map_path <path_to_map>
  // --algorithm <planner_name>
  // --start <start_x> <start_y>
  // --goal <goal_x> <goal_y>
  // --output <output_path (gif file)>

  std::shared_ptr<planning::IPlanning> planner =
      std::make_shared<planning::grid_base::AStar<planning::grid_base::Directions4>>(planning::grid_base::four_directions);

  std::string dataDirectory = DATA_DIR;
  std::string dataFilePath = dataDirectory + "/bg2/AR0072SR.map";

  auto map = std::make_shared<planning::Map>(dataFilePath);

  auto window = sf::RenderWindow{{static_cast<unsigned int>(map->GetWidth()),
                                  static_cast<unsigned int>(map->GetHeight())},
                                 "CMake SFML Project"};
  window.setFramerateLimit(144);

  while (window.isOpen())
    {
      for (auto event = sf::Event{}; window.pollEvent(event);)
        {
          if (event.type == sf::Event::Closed)
            {
              window.close();
            }
        }

      window.clear();

      // Draw your matrix
      drawMatrix(window, map);

      window.display();
    }

  return 0;
}