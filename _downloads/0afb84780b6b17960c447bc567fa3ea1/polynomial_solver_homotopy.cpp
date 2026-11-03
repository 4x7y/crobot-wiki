#include "crobot/ekf/polynomial_solver_homotopy.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;
using namespace ekf;

//==============================================================================
PolynomialSolverHomotopy::HomotopySystem::HomotopySystem(
    PolynomialSystem::ConstPtr system)
  : PolynomialSystem(system->get_dim()), m_f(system) {

  m_g = std::make_shared<PolynomialSystem>(system->get_dim());
  std::vector<int> dk;
  std::vector<double> ck;
  for (int i = 0; i < system->get_dim(); ++i) {
    dk.push_back(2);
    ck.push_back(2);
  }
  m_initial = m_g->initialize(dk, ck);
  m_t       = 0;
}
//==============================================================================
VectorXd PolynomialSolverHomotopy::HomotopySystem::get_init_solution() const {
  return m_initial;
}

//==============================================================================
void PolynomialSolverHomotopy::HomotopySystem::set_t(double t) {
  CROBOT_ASSERT_ERROR(t >= 0 && t <= 1, "t should be in [0, 1]");
  m_t = t;
}
//==============================================================================
VectorXd PolynomialSolverHomotopy::HomotopySystem::h(const VectorXd& x) const {
  CROBOT_ASSERT_ERROR(x.size() == m_f->get_dim(), "x.size() != m_f->get_dim()");

  return m_t * m_f->f(x) + (1 - m_t) * m_g->f(x);
}
//==============================================================================
MatrixXd PolynomialSolverHomotopy::HomotopySystem::get_dhdx(
    const VectorXd& x) const {

  CROBOT_ASSERT_ERROR(x.size() == m_f->get_dim(), "x.size() != m_f->get_dim()");

  MatrixXd dfdx = m_f->dfdx(x);
  MatrixXd dgdx = m_g->dfdx(x);

  return m_t * dfdx + (1 - m_t) * dgdx;
}
//==============================================================================
VectorXd PolynomialSolverHomotopy::HomotopySystem::get_dxdt(
    const VectorXd& x) const {
  CROBOT_ASSERT_ERROR(x.size() == m_f->get_dim(), "x.size() != m_f->get_dim()");

  MatrixXd dhdx = get_dhdx(x);
  VectorXd dhdt = m_f->f(x) - m_g->f(x);

  return dhdx.colPivHouseholderQr().solve(-dhdt);
}

//==============================================================================
PolynomialSolverHomotopy::PolynomialSolverHomotopy(PolynomialSystem::Ptr f) {
  m_system         = std::make_shared<HomotopySystem>(f);
  m_newtons_solver = std::make_shared<const PolynomialSolverNewtons>(m_system);
}

//==============================================================================
VectorXd PolynomialSolverHomotopy::solve(
    int rounds, int polish_rounds, double eps) const {

  m_system->set_t(0);
  VectorXd x = m_system->get_init_solution();

  for (int i = 0; i < rounds; ++i) {

    VectorXd dxdt = m_system->get_dxdt(x);
    x += dxdt * (1.0 / rounds);

    double t = (i + 1) / (double)rounds;

    m_system->set_t(t);

    std::cout << "Homotopy at t = " << t << ", f(x) = " << m_system->h(x).norm()
              << std::endl;

    // polish the solution using Newton's method
    bool converged;
    std::tie(x, converged) = m_newtons_solver->solve(x, polish_rounds, eps);

    std::cout << "after f(x) = " << m_system->h(x).norm() << std::endl;
    std::cout << " ---- " << std::endl;

    if (!converged) {
      break;
    }
  }

  m_system->set_t(1);
  CROBOT_ASSERT_ERROR(m_system->h(x).norm() < eps, "Homotopy did not converge");

  return x;
}
