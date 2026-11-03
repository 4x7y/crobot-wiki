#pragma once

#include "crobot/math/matrix.hpp"
#include "crobot/math/state_space.hpp"

namespace crobot::planning {

class Path {
public:
  Path() = default;

  virtual ~Path();

  virtual math::VectorXd evaluate(double s) = 0;

  virtual std::pair<double, double> get_range() const = 0;

  /**
   * @brief Reduce the vector to @maxSize length
   * @details We do this by sampling to evenly distribute deletions
   *
   * @param states     The vector of T values to sample
   * @param maxSize Max length of the resulting vector
   */
  void down_sample_vector(std::vector<math::VectorXd>& states, size_t maxSize);

  /**
   * @brief Deletes waypoints from @pts
   *
   * @param pts A vector of states that constitutes the path
   * @param transitionValidator A function that returns a boolean indicating
   * whether or not a straight connection exists between the two given states.
   */
  void smooth_path(
      std::vector<math::VectorXd>& pts,
      const math::StateSpace<math::VectorXd>& stateSpace);

private:
};

} // namespace crobot::planning

