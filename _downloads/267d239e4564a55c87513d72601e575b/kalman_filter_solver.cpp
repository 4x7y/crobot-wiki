#include "crobot/ekf/kalman_filter_solver.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;
using namespace ekf;
using namespace optimization;

namespace {
MatrixXd checked_inverse(const MatrixXd& matrix) {
  Eigen::FullPivLU<MatrixXd> decomposition(matrix);
  if (!matrix.allFinite() || !decomposition.isInvertible())
    throw std::runtime_error("Singular constrained Kalman system");
  return decomposition.inverse();
}

void check_shape(const MatrixXd& matrix, int rows, int cols) {
  if (matrix.rows() != rows || matrix.cols() != cols)
    throw std::invalid_argument("Kalman model returned an incorrect dimension");
}
} // namespace

LambdaVariable::LambdaVariable(int dim) : VariableSet(dim, "lambda") {
  m_value = math::VectorXd::Zero(dim);
  bounds.resize(dim);
  for (int i = 0; i < dim; ++i) {
    bounds[i] = Bounds(-5, 5); // -100 ~ 100/ -5 ~ 5
  }
}

void LambdaVariable::set_variables(const math::VectorXd& value) {
  m_value = value;
}

math::VectorXd LambdaVariable::get_values() const {
  return m_value;
}

VecBound LambdaVariable::get_bounds() const {
  return bounds;
}

void LambdaVariable::set_bounds(const math::VectorXd& center, double eps) {
  for (int i = 0; i < m_value.size(); ++i) {
    bounds[i] = Bounds(center[i] - eps, center[i] + eps);
  }
}

//==============================================================================

LambdaConstraint::LambdaConstraint(
    KFConstrainedSystem::Ptr polysystem, math::VectorXd eps)
  : ConstraintSet(polysystem->get_dim(), "lambda_constraints", {"lambda"}),
    m_polysystem(polysystem) {
  int dim = m_polysystem->get_dim();
  CROBOT_ASSERT_ERROR(eps.size() == dim, "eps.size() != dim");
  // m_bounds = VecBound(dim);
  // for (int i = 0; i < dim; ++i) {
  //   m_bounds[i] = Bounds(-eps[i], eps[i]);
  // }
  m_eps   = eps;
  m_scale = math::VectorXd::Ones(dim);
}

math::VectorXd LambdaConstraint::get_values(const math::VectorXd& x) const {
  return m_polysystem->f(x);
}

optimization::VecBound LambdaConstraint::get_bounds() const {
  // rerturn scaled bounds
  VecBound bounds(m_eps.size());
  for (int i = 0; i < m_eps.size(); ++i) {
    bounds[i] = Bounds(-m_eps[i] * m_scale[i], m_eps[i] * m_scale[i]);
  }
  return bounds;
}

void LambdaConstraint::set_scale(const math::VectorXd& scale) {
  m_scale = scale;
}

void LambdaConstraint::fill_jacobian_block(
    Component::Ptr varsets, math::SparseJacobian& jac_block) const {
  math::VectorXd lambda = varsets->get_values();
  MatrixXd dfdx         = m_polysystem->dfdx(lambda);
  for (int i = 0; i < dfdx.rows(); ++i) {
    for (int j = 0; j < dfdx.cols(); ++j) {
      jac_block.coeffRef(i, j) = dfdx(i, j);
    }
  }
}

//==============================================================================

KFConstrainedSystem::KFConstrainedSystem(KalmanFilterSystem::ConstPtr kf_system)
  : PolynomialSystem(kf_system->get_const_dim()),
    m_kf_system(kf_system),
    updated(false) {
}

//==============================================================================

VectorXd KFConstrainedSystem::f(const VectorXd& lambda) const {
  CROBOT_ASSERT_ERROR(updated, "update() should be called first");

  MatrixXd J;
  VectorXd m;
  double eps;
  std::tie(J, m, eps) = get_J_m_eps(lambda);

  MatrixXd J_inv  = checked_inverse(J);
  VectorXd Jinv_m = J_inv * m;

  // construct the constraint vector, one value for each constraint
  VectorXd const_vec(get_dim());
  for (int i = 0; i < get_dim(); ++i) {
    MatrixXd A = m_constraints_A[i];
    VectorXd b = m_constraints_b[i];
    double c   = m_constraints_c[i]; // used in constraint calculation

    const_vec[i]
        = (m_x.transpose() * A * m_x)[0] + (b.transpose() * m_x)[0] + c;
    const_vec[i] += 2 * m_x.transpose() * A * Jinv_m;
    const_vec[i] += Jinv_m.transpose() * A * Jinv_m;
    const_vec[i] += b.transpose() * Jinv_m;
  }

  return const_vec;
}

//==============================================================================

MatrixXd KFConstrainedSystem::dfdx(const VectorXd& lambda) const {
  CROBOT_ASSERT_ERROR(updated, "update() should be called first");

  MatrixXd J;
  VectorXd m;
  double eps;
  std::tie(J, m, eps) = get_J_m_eps(lambda);

  MatrixXd J_inv  = checked_inverse(J);
  VectorXd Jinv_m = J_inv * m;

  // d(J^-1m)/d(lambda) in (n, num_constraints)
  MatrixXd dJinv_m_dlambda(m_kf_system->get_xdim(), get_dim());

  for (int i = 0; i < get_dim(); ++i) {
    MatrixXd A = m_constraints_A[i];
    VectorXd b = m_constraints_b[i];

    VectorXd dm_dlambi     = -eps * (A * m_x + 0.5 * b);
    MatrixXd dJinv_dlambi  = -eps * J_inv * A * J_inv;
    dJinv_m_dlambda.col(i) = dJinv_dlambi * m + J_inv * dm_dlambi;
  }

  // d(alpha)/d(lambda) in (num_constraints, num_constraints)
  MatrixXd dalpha_dlambda(get_dim(), get_dim());
  for (int i = 0; i < get_dim(); ++i) {
    MatrixXd A = m_constraints_A[i];
    VectorXd b = m_constraints_b[i];

    dalpha_dlambda.row(i)
        = 2
          * (m_x.transpose() * A + Jinv_m.transpose() * A + 0.5 * b.transpose())
          * dJinv_m_dlambda;
  }

  return dalpha_dlambda;
}

//==============================================================================

std::tuple<MatrixXd, VectorXd, double> KFConstrainedSystem::get_J_m_eps(
    const VectorXd& lambda) const {

  // NOTE: The same as get_gain() function

  int n = m_kf_system->get_xdim();

  // calculate S
  MatrixXd S = m_H * m_P * m_H.transpose() + m_R;

  MatrixXd S_inv = checked_inverse(S);

  // calculate epsilon
  double eps = (m_err_y.transpose() * S_inv * m_err_y)[0];

  // calculate N(lambda) and M(lambda)
  MatrixXd N = MatrixXd::Zero(n, n);
  MatrixXd M = m_P * m_H.transpose();
  for (int i = 0; i < get_dim(); ++i) {
    MatrixXd A = m_constraints_A[i];
    VectorXd b = m_constraints_b[i];
    (void)m_constraints_c[i]; // Not used in this calculation

    N += lambda[i] * A;
    M -= lambda[i] * (A * m_x + 0.5 * b) * m_err_y.transpose();
  }

  // calculate J
  MatrixXd J = MatrixXd::Identity(n, n) + eps * N;

  // calculate m
  VectorXd m = M * S_inv * m_err_y;

  return std::make_tuple(J, m, eps);
}

//==============================================================================

MatrixXd KFConstrainedSystem::get_gain(const VectorXd& lambda) const {

  int n = m_kf_system->get_xdim();

  // calculate S
  MatrixXd S = m_H * m_P * m_H.transpose() + m_R;

  MatrixXd S_inv = checked_inverse(S);

  // calculate epsilon
  double eps = (m_err_y.transpose() * S_inv * m_err_y)[0];

  // calculate N(lambda) and M(lambda)
  MatrixXd N = MatrixXd::Zero(n, n);
  MatrixXd M = m_P * m_H.transpose();
  for (int i = 0; i < get_dim(); ++i) {
    MatrixXd A = m_constraints_A[i];
    VectorXd b = m_constraints_b[i];
    (void)m_constraints_c[i]; // Not used in this calculation

    N += lambda[i] * A;
    M -= lambda[i] * (A * m_x + 0.5 * b) * m_err_y.transpose();
  }

  // calculate J
  MatrixXd J = MatrixXd::Identity(n, n) + eps * N;

  MatrixXd J_inv = checked_inverse(J);

  // calculate m
  VectorXd m = M * S_inv * m_err_y;

  // NOTE: the gain is calculated as K = (M - N J^-1 m)^T S^-1
  // The same as get_Jm() function until here

  // calculate gain
  MatrixXd K = (M - N * J_inv * m * m_err_y.transpose()) * S_inv;

  return K;
}

//==============================================================================

void KFConstrainedSystem::update(
    const VectorXd& x,
    const VectorXd& u,
    const VectorXd& err_y,
    const MatrixXd& P,
    const MatrixXd& H,
    const MatrixXd& M) {

  CROBOT_ASSERT_ERROR(x.size() == m_kf_system->get_xdim(), "x.size() != xdim");
  CROBOT_ASSERT_ERROR(u.size() == m_kf_system->get_udim(), "u.size() != udim");
  CROBOT_ASSERT_ERROR(
      err_y.size() == m_kf_system->get_ydim(), "y.size() != ydim");
  CROBOT_ASSERT_ERROR(P.rows() == m_kf_system->get_xdim(), "P.rows() != xdim");
  CROBOT_ASSERT_ERROR(P.cols() == m_kf_system->get_xdim(), "P.cols() != xdim");
  CROBOT_ASSERT_ERROR(H.rows() == m_kf_system->get_ydim(), "H.rows() != ydim");
  CROBOT_ASSERT_ERROR(H.cols() == m_kf_system->get_xdim(), "H.cols() != xdim");
  CROBOT_ASSERT_ERROR(M.rows() == m_kf_system->get_ydim(), "M.rows() != ydim");
  CROBOT_ASSERT_ERROR(M.cols() == m_kf_system->get_vdim(), "M.cols() != vdim");

  updated = true;

  m_x     = x;
  m_u     = u;
  m_err_y = err_y;
  m_P     = P;
  m_H     = H;
  m_R     = M * m_kf_system->get_R() * M.transpose(); // R = MRM^T

  std::tie(m_constraints_A, m_constraints_b, m_constraints_c)
      = m_kf_system->get_constraints(x);

  CROBOT_ASSERT_ERROR(
      static_cast<int>(m_constraints_A.size()) == get_dim(),
      "m_constraints_A.size() != get_dim()");
  CROBOT_ASSERT_ERROR(
      static_cast<int>(m_constraints_b.size()) == get_dim(),
      "m_constraints_b.size() != get_dim()");
  CROBOT_ASSERT_ERROR(
      static_cast<int>(m_constraints_c.size()) == get_dim(),
      "m_constraints_c.size() != get_dim()");
}

//==============================================================================
// cost

LambdaCost::LambdaCost(int dim) : CostTerm("lambda_cost", {"lambda"}) {
  m_target = math::VectorXd::Zero(dim);
}

double LambdaCost::get_cost(const math::VectorXd& x) const {
  math::VectorXd diff = m_target - x;
  return diff.squaredNorm();
}

// Documentation inherited
void LambdaCost::fill_jacobian_block(
    Component::Ptr var_set, math::SparseJacobian& jac) const {
  math::VectorXd lambda = var_set->get_values();
  math::VectorXd diff   = m_target - lambda;
  for (int i = 0; i < lambda.size(); ++i) {
    jac.coeffRef(0, i) = -2.0 * diff[i];
  }
}

void LambdaCost::set_target(const math::VectorXd& target) {
  CROBOT_ASSERT_ERROR(
      target.size() == m_target.size(), "target.size() != m_target.size()");
  m_target = target;
}

//==============================================================================
KalmanFilterSolver::KalmanFilterSolver(
    KalmanFilterSystem::Ptr system,
    double dt,
    double max_cpu_time,
    bool linearized_constraints)
  : m_system(system),
    m_dt(dt),
    m_linearized_constraints(linearized_constraints) {
  if (!system || !std::isfinite(dt) || dt <= 0 || !std::isfinite(max_cpu_time)
      || max_cpu_time <= 0)
    throw std::invalid_argument(
        "Kalman solver requires a model and positive time limits");
  auto eps = system->get_c_eps();
  if (eps.size() != system->get_const_dim() || !eps.allFinite()
      || (eps.array() < 0).any())
    throw std::invalid_argument("Invalid Kalman constraint tolerances");
  m_lambda = VectorXd::Zero(system->get_const_dim());
  m_x      = m_system->get_x0();
  m_P      = m_system->get_P0();

  if (m_system->get_const_dim() > 0 && !m_linearized_constraints) {
    init_constraints_solver(max_cpu_time);
  }
}

//==============================================================================
void KalmanFilterSolver::init_constraints_solver(double max_cpu_time) {
  m_constraints_system = std::make_shared<KFConstrainedSystem>(m_system);

  m_nlp = std::make_shared<optimization::Problem>();

  m_lambda_var
      = std::make_shared<LambdaVariable>(m_constraints_system->get_dim());
  m_nlp->add_variable_set(m_lambda_var);
  m_lambda_constraint = std::make_shared<LambdaConstraint>(
      m_constraints_system, m_system->get_c_eps());
  m_nlp->add_constraint_set(m_lambda_constraint);

  m_solver = std::make_shared<optimization::IpoptSolver>();
  m_solver->set_option("linear_solver", "mumps");
  m_solver->set_option("jacobian_approximation", "exact");
  m_solver->set_option("print_level", 0);
  m_solver->set_option("constr_viol_tol", 1e-6);
  m_solver->set_option("tol", 1e-6);
  m_solver->set_option("print_user_options", "no");
  m_solver->set_option("mu_strategy", "adaptive");
  m_solver->set_option("hessian_approximation", "limited-memory");
  m_solver->set_option("fixed_variable_treatment", "make_parameter");
  m_solver->set_option("warm_start_bound_frac", 1e-6);
  m_solver->set_option("warm_start_bound_push", 1e-6);
  m_solver->set_option("warm_start_mult_bound_push", 1e-6);
  m_solver->set_option("warm_start_slack_bound_frac", 1e-6);
  m_solver->set_option("warm_start_slack_bound_push", 1e-6);
  m_solver->set_option("max_cpu_time", max_cpu_time);

  m_lambda = VectorXd::Zero(m_system->get_const_dim());
}

bool KalmanFilterSolver::update(const VectorXd& y, const VectorXd& u) {
  const int n = m_system->get_xdim(), ny = m_system->get_ydim();
  if (y.size() != ny || u.size() != m_system->get_udim())
    throw std::invalid_argument(
        "Kalman measurement or input dimension mismatch");
  if (!y.allFinite() || !u.allFinite())
    return false;

  try {
    const VectorXd w    = VectorXd::Zero(m_system->get_wdim());
    const VectorXd v    = VectorXd::Zero(m_system->get_vdim());
    VectorXd derivative = m_system->f(m_x, u, w);
    MatrixXd A          = m_system->get_dfdx(m_x, u, w);
    MatrixXd L          = m_system->get_dfdw(m_x, u, w);
    check_shape(derivative, n, 1);
    check_shape(A, n, n);
    check_shape(L, n, w.size());
    if (!derivative.allFinite() || !A.allFinite() || !L.allFinite())
      return false;
    VectorXd x         = m_x + m_dt * derivative;
    MatrixXd H         = m_system->get_dhdx(x, v);
    MatrixXd M         = m_system->get_dhdv(x, v);
    VectorXd predicted = m_system->h(x, v);
    check_shape(H, ny, n);
    check_shape(M, ny, v.size());
    check_shape(predicted, ny, 1);
    if (!x.allFinite() || !H.allFinite() || !M.allFinite()
        || !predicted.allFinite())
      return false;

    // Euler transition with continuous-time process noise. The congruence
    // form preserves positive semidefiniteness, unlike Euler integration of P.
    MatrixXd F = MatrixXd::Identity(n, n) + m_dt * A;
    MatrixXd P = F * m_P * F.transpose()
                 + m_dt * L * m_system->get_Q() * L.transpose();
    MatrixXd R = M * m_system->get_R() * M.transpose();
    MatrixXd S = H * P * H.transpose() + R;
    S          = (0.5 * (S + S.transpose())).eval();
    if (!S.allFinite())
      return false;
    Eigen::LDLT<MatrixXd> decomposition(S);
    if (decomposition.info() != Eigen::Success
        || (decomposition.vectorD().array() <= 0).any())
      return false;
    VectorXd residual = m_system->innovation(y, predicted);
    check_shape(residual, ny, 1);
    if (!residual.allFinite())
      return false;
    MatrixXd K      = decomposition.solve(H * P).transpose();
    VectorXd lambda = m_lambda;

    if (m_system->get_const_dim() > 0 && !m_linearized_constraints) {
      m_constraints_system->update(x, u, residual, P, H, M);
      // Respect the requested tolerances and CPU limit; failure is recoverable.
      m_lambda_var->set_variables(lambda);
      m_solver->solve(*m_nlp);
      if (m_solver->get_return_status() != IpoptSolver::SUCCEEDED)
        return false;
      lambda             = m_nlp->get_variable_values();
      VectorXd violation = m_constraints_system->f(lambda);
      if (!violation.allFinite()
          || (violation.cwiseAbs().array()
              > m_system->get_c_eps().array() + 1e-6)
                 .any())
        return false;
      K = m_constraints_system->get_gain(lambda);
    }

    x += K * residual;
    MatrixXd I_KH = MatrixXd::Identity(n, n) - K * H;
    P = (I_KH * P * I_KH.transpose() + K * R * K.transpose()).eval();

    if (m_system->get_const_dim() > 0 && m_linearized_constraints) {
      const int nc   = m_system->get_const_dim();
      bool converged = false;
      // Newton projection with minimum Euclidean correction. A covariance
      // metric becomes singular after an exact projection (e.g. quaternion
      // normalization), so it cannot reliably correct later normal drift.
      // Project the covariance once at the final tangent plane.
      for (int iteration = 0; iteration < 30; ++iteration) {
        VectorXd violation = m_system->c(x);
        MatrixXd C         = m_system->get_dcdx(x);
        check_shape(violation, nc, 1);
        check_shape(C, nc, n);
        if (!violation.allFinite() || !C.allFinite())
          return false;
        MatrixXd projection
            = C.completeOrthogonalDecomposition().pseudoInverse();
        if ((violation.cwiseAbs().array()
             <= m_system->get_c_eps().array() + 1e-10)
                .all()) {
          MatrixXd tangent = MatrixXd::Identity(n, n) - projection * C;
          P                = (tangent * P * tangent.transpose()).eval();
          converged        = true;
          break;
        }
        x -= projection * violation;
        if (!x.allFinite())
          return false;
      }
      if (!converged)
        return false;
    }
    P = (0.5 * (P + P.transpose())).eval();
    if (!x.allFinite() || !P.allFinite()
        || (P.diagonal().array() < -1e-12).any())
      return false;
    // Commit together so rejected measurements leave the filter unchanged.
    m_x      = x;
    m_P      = P;
    m_lambda = lambda;
    return true;
  }
  catch (const std::runtime_error&) {
    return false;
  }
}

void KalmanFilterSolver::reset() {
  MatrixXd reset = m_system->get_P0_reset();
  MatrixXd P     = reset;
  for (int i = 0; i < P.rows(); ++i)
    for (int j = 0; j < P.cols(); ++j) {
      if (reset(i, i) < 0 && reset(j, j) < 0)
        P(i, j) = m_P(i, j);
      else if (reset(i, i) < 0 || reset(j, j) < 0)
        P(i, j) = 0;
    }
  VectorXd x = m_system->reset_state(m_x);
  check_shape(x, m_x.size(), 1);
  if (!x.allFinite())
    throw std::invalid_argument("Invalid Kalman reset state");
  m_x = x;
  m_P = P;
  m_lambda.setZero();
}
