#include "crobot/ekf/kalman_filter_system.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;
using namespace ekf;
//==============================================================================
KalmanFilterSystem::KalmanFilterSystem(
    const math::VectorXd& x0,
    const math::VectorXd& u0,
    const math::MatrixXd& P0,
    const math::MatrixXd& Q,
    const math::MatrixXd& R,
    const math::MatrixXd& P0_reset,
    int const_dim)
  : m_x0(x0),
    m_u0(u0),
    m_P0(P0),
    m_Q(Q),
    m_R(R),
    m_P0_reset(P0_reset),
    m_const_dim(const_dim) {
}

//==============================================================================
VectorXd KalmanFilterSystem::get_x0() const {
  return m_x0;
}

//==============================================================================
VectorXd KalmanFilterSystem::get_u0() const {
  return m_u0;
}

//==============================================================================
MatrixXd KalmanFilterSystem::get_P0() const {
  return m_P0;
}

//==============================================================================
MatrixXd KalmanFilterSystem::get_Q() const {
  return m_Q;
}

//==============================================================================
MatrixXd KalmanFilterSystem::get_R() const {
  return m_R;
}

//==============================================================================
int KalmanFilterSystem::get_xdim() const {
  return m_x0.size();
}

//==============================================================================
int KalmanFilterSystem::get_udim() const {
  return m_u0.size();
}

//==============================================================================
int KalmanFilterSystem::get_wdim() const {
  return m_Q.rows();
}

//==============================================================================
int KalmanFilterSystem::get_vdim() const {
  return m_R.rows();
}

//==============================================================================
int KalmanFilterSystem::get_ydim() const {
  return get_vdim();
}

//==============================================================================
int KalmanFilterSystem::get_const_dim() const {
  return m_const_dim;
}

//==============================================================================
math::MatrixXd KalmanFilterSystem::get_P0_reset() const {
  return m_P0_reset;
}

//==============================================================================
bool KalmanFilterSystem::verify_diff(
    const math::VectorXd& x,
    const math::VectorXd& u,
    const math::VectorXd& w,
    const math::VectorXd& v) const {

  CROBOT_ASSERT_ERROR(x.size() == get_xdim(), "x.size() != get_xdim()");
  CROBOT_ASSERT_ERROR(u.size() == get_udim(), "u.size() != get_udim()");
  CROBOT_ASSERT_ERROR(w.size() == get_wdim(), "w.size() != get_wdim()");
  CROBOT_ASSERT_ERROR(v.size() == get_vdim(), "v.size() != get_vdim()");

  double eps       = 1e-3;
  double error_tol = 1e-3;
  bool result      = true;

  MatrixXd diff = MatrixXd::Zero(get_xdim(), get_xdim());

  //---------------------------------------------------------------------------
  // verify df/dx
  std::cout << "[checking df/dx]" << std::endl;

  SparseJacobian A = get_dfdx(x, u, w);
  for (int i = 0; i < get_xdim(); ++i) {
    VectorXd x_dot_plus  = f(x + eps * VectorXd::Unit(get_xdim(), i), u, w);
    VectorXd x_dot_minus = f(x - eps * VectorXd::Unit(get_xdim(), i), u, w);
    diff.col(i)          = (x_dot_plus - x_dot_minus) / (2 * eps);
  }

  // check diff element-wise
  for (int i = 0; i < get_xdim(); ++i) {
    for (int j = 0; j < get_xdim(); ++j) {
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

  //---------------------------------------------------------------------------
  // verify df/du
  std::cout << "[checking df/dw]" << std::endl;

  SparseJacobian L = get_dfdw(x, u, w);
  diff             = MatrixXd::Zero(get_xdim(), get_wdim());
  for (int i = 0; i < get_wdim(); ++i) {
    VectorXd x_dot_plus  = f(x, u, w + eps * VectorXd::Unit(get_wdim(), i));
    VectorXd x_dot_minus = f(x, u, w - eps * VectorXd::Unit(get_wdim(), i));
    diff.col(i)          = (x_dot_plus - x_dot_minus) / (2 * eps);
  }

  // check diff element-wise
  for (int i = 0; i < get_xdim(); ++i) {
    for (int j = 0; j < get_wdim(); ++j) {
      if (std::abs(L.coeff(i, j) - diff(i, j)) > error_tol) {
        std::cout << "[Error] df/dw(" << i << ", " << j
                  << "): " << L.coeff(i, j) << " != " << diff(i, j)
                  << std::endl;
        result = false;
      }
    }
  }
  std::cout << std::endl << std::endl;

  //---------------------------------------------------------------------------
  // verify dh/dx
  std::cout << "[checking dh/dx]" << std::endl;

  SparseJacobian C = get_dhdx(x, v);
  diff             = MatrixXd::Zero(get_ydim(), get_xdim());
  for (int i = 0; i < get_xdim(); ++i) {
    VectorXd y_plus  = h(x + eps * VectorXd::Unit(get_xdim(), i), v);
    VectorXd y_minus = h(x - eps * VectorXd::Unit(get_xdim(), i), v);
    diff.col(i)      = (y_plus - y_minus) / (2 * eps);
  }

  // check diff element-wise
  for (int i = 0; i < get_ydim(); ++i) {
    for (int j = 0; j < get_xdim(); ++j) {
      if (std::abs(C.coeff(i, j) - diff(i, j)) > error_tol) {
        std::cout << "[Error] dh/dx(" << i << ", " << j
                  << "): " << C.coeff(i, j) << " != " << diff(i, j)
                  << std::endl;
        result = false;
      }
    }
  }
  std::cout << std::endl << std::endl;

  //---------------------------------------------------------------------------
  // verify dh/dv
  std::cout << "[checking dh/dv]" << std::endl;

  SparseJacobian M = get_dhdv(x, v);
  diff             = MatrixXd::Zero(get_ydim(), get_vdim());
  for (int i = 0; i < get_vdim(); ++i) {
    VectorXd y_plus  = h(x, v + eps * VectorXd::Unit(get_vdim(), i));
    VectorXd y_minus = h(x, v - eps * VectorXd::Unit(get_vdim(), i));
    diff.col(i)      = (y_plus - y_minus) / (2 * eps);
  }

  // check diff element-wise
  for (int i = 0; i < get_ydim(); ++i) {
    for (int j = 0; j < get_vdim(); ++j) {
      if (std::abs(M.coeff(i, j) - diff(i, j)) > error_tol) {
        std::cout << "[Error] dh/dv(" << i << ", " << j
                  << "): " << M.coeff(i, j) << " != " << diff(i, j)
                  << std::endl;
        result = false;
      }
    }
  }
  std::cout << std::endl << std::endl;

  //---------------------------------------------------------------------------
  // verify dc/dx
  std::cout << "[checking dc/dx]" << std::endl;

  SparseJacobian dc_dx = get_dcdx(x);

  diff = MatrixXd::Zero(dc_dx.rows(), get_xdim());
  for (int i = 0; i < get_xdim(); ++i) {
    VectorXd c_plus  = c(x + eps * VectorXd::Unit(get_xdim(), i));
    VectorXd c_minus = c(x - eps * VectorXd::Unit(get_xdim(), i));
    diff.col(i)      = (c_plus - c_minus) / (2 * eps);
  }

  // check diff element-wise
  for (int i = 0; i < dc_dx.rows(); ++i) {
    for (int j = 0; j < get_xdim(); ++j) {
      if (std::abs(dc_dx.coeff(i, j) - diff(i, j)) > error_tol) {
        std::cout << "[Error] dc/dx(" << i << ", " << j
                  << "): " << dc_dx.coeff(i, j) << " != " << diff(i, j)
                  << std::endl;
        result = false;
      }
    }
  }

  return result;
}
