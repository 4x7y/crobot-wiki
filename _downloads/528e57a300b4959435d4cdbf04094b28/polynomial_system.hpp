#pragma once

#include "crobot/math/matrix.hpp"

#include <memory>

namespace crobot::ekf {

/**
 * @brief Description of a continuous-time dynamic system with discrete-time
 * measurements for hybrid extended Kalman filter. (P405)
 */
class PolynomialSystem {
public:
  using Ptr      = std::shared_ptr<PolynomialSystem>;
  using ConstPtr = std::shared_ptr<const PolynomialSystem>;

  /**
   * @brief Construct a new Polynomial System object for Newton's method/
   * Homotopy method in the format of f(x) = [f_1(x); f_2(x); ...; f_m(x)]
   *
   * @param dim Dimension of the system
   */
  PolynomialSystem(int dim);

  /**
   * @brief Initialize the system as f_k(x) = x_k^d_k - c_k, so that the
   * solution is x = c^(1/d)
   *
   * @param dk
   * @param ck
   */
  math::VectorXd initialize(std::vector<int> dk, std::vector<double> ck);

  virtual math::VectorXd f(const math::VectorXd& x) const;

  virtual math::MatrixXd dfdx(const math::VectorXd& x) const;

  int get_dim() const;

  bool verify_diff(const math::VectorXd& x) const;

private:
  int m_dim;

  std::vector<int> m_dk;
  std::vector<double> m_ck;
};

} // namespace crobot::ekf

