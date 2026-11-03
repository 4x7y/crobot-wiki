#pragma once

#include "crobot/math/matrix.hpp"

#include <memory>

namespace crobot::ekf {

/**
 * @brief Description of a continuous-time dynamic system with discrete-time
 * measurements for hybrid extended Kalman filter. (P405)
 */
class KalmanFilterSystem {
public:
  using Ptr      = std::shared_ptr<KalmanFilterSystem>;
  using ConstPtr = std::shared_ptr<const KalmanFilterSystem>;

  /**
   * @brief Construct a new Kalman Filter System object
   *
   * @param x0 The initial state vector
   * @param u0 The input vector
   * @param P0 The initial state covariance matrix
   * @param Q The process w noise covariance matrix
   * @param R The measurement v noise covariance matrix
   * @param const_dim The number of constraints for the system
   */
  KalmanFilterSystem(
      const math::VectorXd& x0,
      const math::VectorXd& u0,
      const math::MatrixXd& P0,
      const math::MatrixXd& Q,
      const math::MatrixXd& R,
      const math::MatrixXd& P0_reset,
      int const_dim);

  math::VectorXd get_x0() const;

  math::VectorXd get_u0() const;

  math::MatrixXd get_P0() const;

  math::MatrixXd get_Q() const;

  math::MatrixXd get_R() const;

  int get_xdim() const;

  int get_udim() const;

  int get_wdim() const;

  int get_vdim() const;

  int get_ydim() const;

  int get_const_dim() const;

  math::MatrixXd get_P0_reset() const;
  /**
   * @brief Continuous-time state transition function
   * f(x, u, w) = x_dot
   * @param x State vector
   * @param u Input vector
   * @param w Process noise vector
   */
  virtual math::VectorXd f(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w) const = 0;

  /**
   * @brief Discrete-time measurement function
   * h(x, v) = y
   * @param x State vector
   * @param v Measurement noise vector
   */
  virtual math::VectorXd h(
      const math::VectorXd& x, const math::VectorXd& v) const = 0;

  /**
   * @brief Constraints for the system in the form of c(x) = 0
   *
   * @param x State vector
   */
  virtual math::VectorXd c(const math::VectorXd& x) const = 0;

  /**
   * @brief Get the c_eps vector for the constraints
   *
   * @return math::VectorXd
   */
  virtual math::VectorXd get_c_eps() const = 0;

  /**
   * @brief Jacobian of the constraints dc/dx = A
   *
   * @param x State vector
   */
  virtual math::SparseJacobian get_dcdx(const math::VectorXd& x) const = 0;

  /**
   * @brief Jacobian of the state transition function
   * df/dx = A
   * @param x State vector
   * @param u Input vector
   * @param w Process noise vector
   */
  virtual math::SparseJacobian get_dfdx(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w) const = 0;

  /**
   * @brief Jacobian of the state transition function
   * df/dw = L
   * @param x State vector
   * @param u Input vector
   * @param w Process noise vector
   */
  virtual math::SparseJacobian get_dfdw(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w) const = 0;

  /**
   * @brief Jacobian of the measurement function
   * dh/dx = C
   * @param x State vector
   * @param v Measurement noise vector
   */
  virtual math::SparseJacobian get_dhdx(
      const math::VectorXd& x, const math::VectorXd& v) const = 0;

  /**
   * @brief Jacobian of the measurement function
   * dh/dv = M
   * @param x State vector
   * @param v Measurement noise vector
   */
  virtual math::SparseJacobian get_dhdv(
      const math::VectorXd& x, const math::VectorXd& u) const = 0;

  /**
   * @brief Get the constraints set for the system in the form of
   * x^T A x + b^T x + c = 0
   *
   * @param x State vector
   * @return std::vector<std::tuple<math::MatrixXd, math::VectorXd, double>>
   * A vector of tuples containing the matrices A, b, and c
   */
  virtual std::tuple<
      std::vector<math::MatrixXd>,
      std::vector<math::VectorXd>,
      std::vector<double>>
  get_constraints(const math::VectorXd& x) const = 0;

  /**
   * @brief Get the constraints set for the system in the form of
   * A x = b
   *
   * @param x State vector
   * @return std::tuple<math::MatrixXd, math::VectorXd> A tuple containing the
   * matrices A and b
   */
  virtual std::tuple<math::MatrixXd, math::VectorXd> get_linear_constraints(
      const math::VectorXd& x) const = 0;

  bool verify_diff(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w,
      const math::VectorXd& v) const;

private:
  math::VectorXd m_x0;
  math::VectorXd m_u0;
  math::MatrixXd m_P0;
  math::MatrixXd m_Q;
  math::MatrixXd m_R;

  math::MatrixXd m_P0_reset;

  int m_const_dim;
};

} // namespace crobot::ekf

