#pragma once

#include <memory>

#include "crobot/math/matrix.hpp"

namespace crobot::planning {

class StatePropagator {
public:
  using Ptr      = std::shared_ptr<StatePropagator>;
  using ConstPtr = std::shared_ptr<const StatePropagator>;

  StatePropagator(double step_size, int num_output)
    : m_step_size(step_size), m_num_output(num_output) {
  }

  virtual ~StatePropagator() = default;

  /**
   * @brief Propagate system and return valid motion
   *
   * @param state_current
   * @param control
   * @param state_target
   * @param max_steps
   * @return [q, u, steps]
   *   q      The final state of the system
   *   u      The control after final step has been reached
   *   steps  The total number of steps between (q0,q)
   */
  // virtual std::tuple<> propagate_while_valid(
  //     const math::VectorXd& state_current,
  //     const Control& control,
  //     const math::VectorXd& state_target,
  //     int max_steps)
  //     = 0;

  /**
   * @brief Estimated cost of propagating to target state
   *
   * H = distance(Q1, q2) returns the estimated distance between a set of
   * initial states, Q1 as an N-by-S matrix, and a final state, q2 as an
   * S-element row vector, where N is the number of states and S is the number
   * of variables in the StateSpace.
   *
   * For each {q1,q2} pair, this function should return the estimated cost of
   * propagating the system from q1 to q2.
   *
   * @note Values returned by this function are used to find the
   * "nearest"-neighbor to a sampled target state.
   */
  virtual double distance(const math::VectorXd& s1, const math::VectorXd& s2)
      = 0;

  /**
   * @brief Generate a control command and duration
   *
   * [u, steps] = sample_control(obj, q0, u0, qTgt) Takes in the current state
   * and control command, q0, u0, respectively, and generates a new control
   * command, u, and number of steps to propagate the system, towards the target
   * state, qTgt.
   *
   * @param s1
   * @param s2
   * @return std::tuple<>
   */
  virtual std::tuple<> sample_control(
      const math::VectorXd& s1, const math::VectorXd& s2)
      = 0;

  /**
   * @brief A setup utility called at the start of planning
   *
   * setup(OBJ) Used to perform initial configurations to the StatePropagator
   * object. This method is always called by the planner at the beginning of the
   * 'plan' method.
   */
  virtual void setup() {
  }

private:
  double m_step_size;

  // Number of variables in control command
  int m_num_output;
};

} // namespace crobot::planning

