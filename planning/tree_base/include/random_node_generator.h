#include <planning/include/data_types.h>
#include <planning/include/random_number_generator.h>

namespace planning
{
namespace tree_base
{
template <int min, int max> class RandomNodeGenerator final
{
public:
  Node operator()() { return Node{rig_(), rig_()}; }

private:
  RandomIntGenerator<min, max> rig_{};
};
} // namespace tree_base
} // namespace planning