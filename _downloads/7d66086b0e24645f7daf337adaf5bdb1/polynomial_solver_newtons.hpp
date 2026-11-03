#pragma once

#include "crobot/ekf/polynomial_system.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::ekf {

/**
 * @brief Solve a system of polynomial equations using Newton's method.
 */
class PolynomialSolverNewtons {
public:
  using Ptr      = std::shared_ptr<PolynomialSolverNewtons>;
  using ConstPtr = std::shared_ptr<const PolynomialSolverNewtons>;

  PolynomialSolverNewtons(PolynomialSystem::Ptr system);

  std::pair<math::VectorXd, bool> solve(
      const math::VectorXd& initial_guess,
      int rounds = 100,
      double eps = 1e-5) const;

private:
  PolynomialSystem::Ptr m_system;
};

} // namespace crobot::ekf

