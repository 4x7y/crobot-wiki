#include "crobot/ekf/polynomial_system.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;
using namespace ekf;
//==============================================================================
PolynomialSystem::PolynomialSystem(int dim) : m_dim(dim) {
}

//==============================================================================
int PolynomialSystem::get_dim() const {
  return m_dim;
}

//==============================================================================
VectorXd PolynomialSystem::initialize(
    std::vector<int> dk, std::vector<double> ck) {
  CROBOT_ASSERT_ERROR(dk.size() == ck.size(), "dk.size() != ck.size()");
  CROBOT_ASSERT_ERROR(
      static_cast<int>(dk.size()) == m_dim, "dk.size() != m_dim");
  // make sure that dk and ck are non-negative by checking the minimum value
  CROBOT_ASSERT_ERROR(
      *std::min_element(dk.begin(), dk.end()) > 0, "dk should be non-negative");
  CROBOT_ASSERT_ERROR(
      *std::min_element(ck.begin(), ck.end()) > 0, "ck should be non-negative");

  m_dk = dk;
  m_ck = ck;

  VectorXd x = VectorXd::Zero(m_dim);
  for (int i = 0; i < m_dim; ++i) {
    x(i) = std::pow(ck[i], 1.0 / dk[i]);
  }

  return x;
}
//==============================================================================
VectorXd PolynomialSystem::f(const math::VectorXd& x) const {
  CROBOT_ASSERT_ERROR(
      static_cast<int>(m_dk.size()) == m_dim, "m_dk not initialized");
  CROBOT_ASSERT_ERROR(x.size() == get_dim(), "x.size() != get_dim()");

  VectorXd y = VectorXd::Zero(get_dim());
  for (int i = 0; i < get_dim(); ++i) {
    y(i) = std::pow(x(i), m_dk[i]) - m_ck[i];
  }

  return y;
}
//==============================================================================
MatrixXd PolynomialSystem::dfdx(const math::VectorXd& x) const {
  CROBOT_ASSERT_ERROR(x.size() == get_dim(), "x.size() != get_dim()");

  MatrixXd A(get_dim(), get_dim());
  for (int i = 0; i < get_dim(); ++i) {
    A.coeffRef(i, i) = m_dk[i] * std::pow(x(i), m_dk[i] - 1);
  }

  return A;
}

//==============================================================================
bool PolynomialSystem::verify_diff(const math::VectorXd& x) const {

  CROBOT_ASSERT_ERROR(x.size() == get_dim(), "x.size() != get_dim()");
  CROBOT_ASSERT_ERROR(f(x).size() == get_dim(), "f(x).size() != get_dim()");
  CROBOT_ASSERT_ERROR(
      dfdx(x).rows() == get_dim(), "dfdx(x).rows() != get_dim()");
  CROBOT_ASSERT_ERROR(
      dfdx(x).cols() == get_dim(), "dfdx(x).cols() != get_dim()");

  double eps       = 1e-3;
  double error_tol = 1e-3;
  bool result      = true;

  MatrixXd diff = MatrixXd::Zero(get_dim(), get_dim());

  //---------------------------------------------------------------------------
  // verify df/dx
  std::cout << "[checking df/dx]" << std::endl;

  MatrixXd A = dfdx(x);
  for (int i = 0; i < get_dim(); ++i) {
    VectorXd x_dot_plus  = f(x + eps * VectorXd::Unit(get_dim(), i));
    VectorXd x_dot_minus = f(x - eps * VectorXd::Unit(get_dim(), i));
    diff.col(i)          = (x_dot_plus - x_dot_minus) / (2 * eps);
  }

  // check diff element-wise
  for (int i = 0; i < get_dim(); ++i) {
    for (int j = 0; j < get_dim(); ++j) {
      if (std::abs(A.coeff(i, j) - diff(i, j)) / std::abs(diff(i, j))
              > error_tol
          && std::abs(A.coeff(i, j) - diff(i, j)) > error_tol) {
        std::cout << "[Error] df/dx(" << i << ", " << j
                  << "): " << A.coeff(i, j) << " != " << diff(i, j)
                  << std::endl;
        result = false;
      }
    }
  }
  std::cout << std::endl << std::endl;

  return result;
}
