#include "crobot/planning/path.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
void Path::down_sample_vector(
    std::vector<math::VectorXd>& states, size_t maxSize) {
  if (states.size() > maxSize) {
    int toDelete   = states.size() - maxSize;
    double spacing = (double)states.size() / (double)toDelete;
    double i       = 0.0;
    while (toDelete) {
      toDelete--;
      states.erase(states.begin() + (int)(i + 0.5));
      i += spacing - 1.0;
    }
  }
}

//==============================================================================
void Path::smooth_path(
    std::vector<math::VectorXd>& pts,
    const math::StateSpace<math::VectorXd>& stateSpace) {
  std::size_t span = 2;
  while (span < pts.size()) {
    bool changed = false;
    for (std::size_t i = 0; i + span < pts.size(); i++) {
      if (stateSpace.is_transition_valid(pts[i], pts[i + span])) {
        for (std::size_t x = 1; x < span; x++) {
          pts.erase(pts.begin() + i + 1);
        }
        changed = true;
      }
    }

    if (!changed)
      span++;
  }
}
