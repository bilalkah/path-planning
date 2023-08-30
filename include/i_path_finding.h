/**
 * @file i_path_finding.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Interface for path finding algorithms.
 * @version 0.1
 * @date 2023-08-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_I_PATH_FINDING_H_
#define INCLUDE_I_PATH_FINDING_H_

#include <memory>

class Map;
class Node;

class IPathFinding
{
public:
  IPathFinding();

  virtual ~IPathFinding();

  virtual void FindPath(Node &start_node, Node &goal_node,
                        std::shared_ptr<Map> map) = 0;
};

#endif /* INCLUDE_I_PATH_FINDING_H_ */