#pragma once

#include "crobot/ekf/polynomial_solver_newtons.hpp"
#include "crobot/ekf/polynomial_system.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ekf {

/**
 * @brief Solve a system of polynomial equations using homotopy continuation.
 */
class PolynomialSolverHomotopy {
public:
  using Ptr      = std::shared_ptr<PolynomialSolverHomotopy>;
  using ConstPtr = std::shared_ptr<const PolynomialSolverHomotopy>;

  PolynomialSolverHomotopy(PolynomialSystem::Ptr f);

  math::VectorXd solve(
      int rounds = 200, int polish_rounds = 50, double eps = 1e-4) const;

private:
  class HomotopySystem : public PolynomialSystem {
  public:
    using Ptr = std::shared_ptr<HomotopySystem>;

    HomotopySystem(PolynomialSystem::ConstPtr system);
    math::VectorXd get_init_solution() const;
    math::VectorXd h(const math::VectorXd& x) const;
    math::MatrixXd get_dhdx(const math::VectorXd& x) const;
    math::VectorXd get_dxdt(const math::VectorXd& x) const;

    void set_t(double t);

    math::VectorXd f(const math::VectorXd& x) const override {
      return h(x);
    }

    math::MatrixXd dfdx(const math::VectorXd& x) const override {
      return get_dhdx(x);
    }

  private:
    PolynomialSystem::ConstPtr m_f;
    PolynomialSystem::Ptr m_g;
    math::VectorXd m_initial;
    double m_t;
  };

  HomotopySystem::Ptr m_system;
  PolynomialSolverNewtons::ConstPtr m_newtons_solver;
};

} // namespace crobot::ekf

