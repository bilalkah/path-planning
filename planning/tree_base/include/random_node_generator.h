#include <planning/include/data_types.h>
#include <planning/include/random_number_generator.h>

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