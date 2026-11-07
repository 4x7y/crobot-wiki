#pragma once

#include "crobot/ekf/kalman_filter_system.hpp"

namespace crobot::ekf {

/// Recursive estimation of a static state: no control or process noise.
/// Observation and constraint models use the same interface as an EKF.
class LeastSquareSystem : public KalmanFilterSystem {
public:
  using Ptr      = std::shared_ptr<LeastSquareSystem>;
  using ConstPtr = std::shared_ptr<const LeastSquareSystem>;

  LeastSquareSystem(
      const math::VectorXd& x0,
      const math::MatrixXd& P0,
      const math::MatrixXd& R,
      int const_dim)
    : KalmanFilterSystem(
        x0, math::VectorXd(0), P0, math::MatrixXd(0, 0), R, P0, const_dim) {
  }

  math::VectorXd f(
      const math::VectorXd&,
      const math::VectorXd&,
      const math::VectorXd&) const override {
    return math::VectorXd::Zero(get_xdim());
  }
  math::SparseJacobian get_dfdx(
      const math::VectorXd&,
      const math::VectorXd&,
      const math::VectorXd&) const override {
    return math::SparseJacobian(get_xdim(), get_xdim());
  }
  math::SparseJacobian get_dfdw(
      const math::VectorXd&,
      const math::VectorXd&,
      const math::VectorXd&) const override {
    return math::SparseJacobian(get_xdim(), 0);
  }
};

} // namespace crobot::ekf
