#include "crobot/ekf/polynomial_solver_newtons.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;
using namespace ekf;

//==============================================================================
PolynomialSolverNewtons::PolynomialSolverNewtons(PolynomialSystem::Ptr system)
  : m_system(system) {
}

//==============================================================================
std::pair<VectorXd, bool> PolynomialSolverNewtons::solve(
    const VectorXd& initial_guess, int rounds, double eps) const {
  CROBOT_ASSERT_ERROR(
      initial_guess.size() == m_system->get_dim(),
      "initial_guess.size() != m_system->get_dim()");

  VectorXd x = initial_guess;

  for (int i = 0; i < rounds; ++i) {
    VectorXd f = m_system->f(x);
    MatrixXd A = m_system->dfdx(x);

    VectorXd dx = A.colPivHouseholderQr().solve(-f);

    x += dx;

    // find the maximum absolute value of the elements of f(x)
    // double max_fx = m_system->f(x).cwiseAbs().maxCoeff();

    if (dx.norm() < eps) {
      return {x, true};
    }
  }

  // std::cout << "Newton's method did not converge, final result f(x) = "
  //           << m_system->f(x).transpose() << std::endl;

  return {x, false};
}
