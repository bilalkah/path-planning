#include <planning/include/data_types.h>
#include <planning/include/random_number_generator.h>

#ifndef PLANNING_TREE_BASE_RANDOM_NODE_GENERATOR_H_
#define PLANNING_TREE_BASE_RANDOM_NODE_GENERATOR_H_

namespace planning
{
namespace tree_base
{
class RandomNodeGenerator final
{
public:
  RandomNodeGenerator(int const min, int const max) : rig_{min, max} {}
  Node operator()() { return Node{rig_(), rig_()}; }

private:
  RandomIntGenerator rig_{0, 1};
};
} // namespace tree_base
} // namespace planning

#endif /* PLANNING_TREE_BASE_RANDOM_NODE_GENERATOR_H_ */