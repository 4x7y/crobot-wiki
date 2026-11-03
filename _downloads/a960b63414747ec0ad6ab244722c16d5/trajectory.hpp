#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::planning {

class Trajectory {
public:
  using Ptr             = std::shared_ptr<Trajectory>;
  using ConstPtr        = std::shared_ptr<const Trajectory>;
  virtual ~Trajectory() = default;

  /**
   * @brief Get the duration of the trajectory.
   *
   * @return double The duration of the trajectory.
   */
  virtual double get_duration() const = 0;

  /**
   * @brief Get the position of the trajectory at a given time.
   *
   * @param time The time at which to get the position.
   * @return math::VectorXd The position at the given time.
   */
  virtual math::VectorXd get_position(double time) const = 0;

  /**
   * @brief Get the velocity of the trajectory at a given time.
   *
   * @param time The time at which to get the velocity.
   * @return math::VectorXd The velocity at the given time.
   */
  virtual math::VectorXd get_velocity(double time) const = 0;
};

} // namespace crobot::planning

