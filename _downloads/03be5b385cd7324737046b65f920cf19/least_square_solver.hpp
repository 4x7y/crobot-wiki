#pragma once

#include "crobot/ekf/kalman_filter_solver.hpp"
#include "crobot/ekf/least_square_system.hpp"

namespace crobot::ekf {

/// Static-state specialization of the shared (constrained) Kalman update.
class LeastSquareSolver : public KalmanFilterSolver {
public:
  using Ptr = std::shared_ptr<LeastSquareSolver>;

  explicit LeastSquareSolver(
      LeastSquareSystem::Ptr system,
      bool linearized_constraints = false,
      double max_cpu_time         = 0.1)
    : KalmanFilterSolver(system, 1.0, max_cpu_time, linearized_constraints) {
  }

  bool update(const math::VectorXd& measurement) {
    return KalmanFilterSolver::update(measurement, math::VectorXd(0));
  }
};

} // namespace crobot::ekf
