#include "crobot/ekf/kalman_filter_solver.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;
using namespace ekf;
using namespace optimization;

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

  MatrixXd J_inv  = math::calculate_jacb_inv(J);
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

  CROBOT_ASSERT_ERROR(J.determinant() != 0, "J is singular");

  MatrixXd J_inv  = math::calculate_jacb_inv(J);
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

  CROBOT_ASSERT_ERROR(S.determinant() != 0, "S is singular");

  MatrixXd S_inv = math::calculate_jacb_inv(S);

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

  CROBOT_ASSERT_ERROR(J.determinant() != 0, "J is singular");

  return std::make_tuple(J, m, eps);
}

//==============================================================================

MatrixXd KFConstrainedSystem::get_gain(const VectorXd& lambda) const {

  int n = m_kf_system->get_xdim();

  // calculate S
  MatrixXd S = m_H * m_P * m_H.transpose() + m_R;

  CROBOT_ASSERT_ERROR(S.determinant() != 0, "S is singular");

  MatrixXd S_inv = math::calculate_jacb_inv(S);

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

  CROBOT_ASSERT_ERROR(J.determinant() != 0, "J is singular");

  MatrixXd J_inv = math::calculate_jacb_inv(J);

  // calculate m
  VectorXd m = M * S_inv * m_err_y;

  // NOTE: the gain is calculated as K = (M - N J^-1 m)^T S^-1
  // The same as get_Jm() function until here

  // calculate gain
  MatrixXd K = (M - N * J_inv * m * m_err_y.transpose()) * S_inv;

  if (K.hasNaN()) {
    std::cout << "K has NaN" << std::endl;
    std::cout << "M = " << std::endl << M << std::endl;
    std::cout << "N = " << std::endl << N << std::endl;
    std::cout << "J = " << std::endl << J << std::endl;
    std::cout << "J_inv = " << std::endl << J_inv << std::endl;
    std::cout << "S_inv = " << std::endl << S_inv << std::endl;
    std::cout << "m = " << m.transpose() << std::endl;
    std::cout << "err_y = " << m_err_y.transpose() << std::endl;
  }

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
  m_x = m_system->get_x0();
  m_P = m_system->get_P0();

  if (m_system->get_const_dim() > 0) {
    init_constraints_solver(max_cpu_time);
  }
}

//==============================================================================
void KalmanFilterSolver::init_constraints_solver(double max_cpu_time
                                                 [[maybe_unused]]) {
  m_constraints_system = std::make_shared<KFConstrainedSystem>(m_system);

  m_solver_homotopy
      = std::make_shared<PolynomialSolverHomotopy>(m_constraints_system);
  m_solver_newtons
      = std::make_shared<PolynomialSolverNewtons>(m_constraints_system);

  m_nlp = std::make_shared<optimization::Problem>();

  m_lambda_var
      = std::make_shared<LambdaVariable>(m_constraints_system->get_dim());
  m_nlp->add_variable_set(m_lambda_var);
  m_lambda_constraint = std::make_shared<LambdaConstraint>(
      m_constraints_system, m_system->get_c_eps());
  m_nlp->add_constraint_set(m_lambda_constraint);

  // cost to guide the optimization
  m_lambda_cost = std::make_shared<LambdaCost>(m_constraints_system->get_dim());
  // m_nlp->add_cost_set(m_lambda_cost);

  m_solver = std::make_shared<optimization::IpoptSolver>();
  m_solver->set_option("linear_solver", "mumps");
  // m_solver->set_option("jacobian_approximation", "finite-difference-values");
  m_solver->set_option("jacobian_approximation", "exact");
  // m_solver->set_option("derivative_test", "first-order");
  // if (verbose) {
  //   m_solver->set_option("print_level", 4);
  // }
  // else {
  m_solver->set_option("print_level", 0);
  // }
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
  m_solver->set_option("max_cpu_time", 1.0); // 0.04

  m_lambda = VectorXd::Zero(m_system->get_const_dim());
}

//==============================================================================
bool KalmanFilterSolver::update(const VectorXd& y, const VectorXd& u) {

  // step 1: prediction (equ. 13.32)
  VectorXd x_dot = m_system->f(m_x, u, VectorXd::Zero(m_system->get_wdim()));

  SparseJacobian A
      = m_system->get_dfdx(m_x, u, VectorXd::Zero(m_system->get_wdim()));
  SparseJacobian L
      = m_system->get_dfdw(m_x, u, VectorXd::Zero(m_system->get_wdim()));

  CROBOT_ASSERT_ERROR(A.rows() == m_system->get_xdim(), "A.rows() != x.size()");
  CROBOT_ASSERT_ERROR(A.cols() == m_system->get_xdim(), "A.cols() != x.size()");
  CROBOT_ASSERT_ERROR(L.rows() == m_system->get_xdim(), "L.rows() != x.size()");
  CROBOT_ASSERT_ERROR(L.cols() == m_system->get_wdim(), "L.cols() != w.size()");

  VectorXd x = m_x + x_dot * m_dt; // x_k^-
  SparseJacobian H
      = m_system->get_dhdx(x, VectorXd::Zero(m_system->get_vdim()));
  SparseJacobian M
      = m_system->get_dhdv(x, VectorXd::Zero(m_system->get_vdim()));

  CROBOT_ASSERT_ERROR(H.rows() == m_system->get_ydim(), "H.rows() != y.size()");
  CROBOT_ASSERT_ERROR(H.cols() == m_system->get_xdim(), "H.cols() != x.size()");
  CROBOT_ASSERT_ERROR(M.rows() == m_system->get_ydim(), "M.rows() != y.size()");
  CROBOT_ASSERT_ERROR(M.cols() == m_system->get_vdim(), "M.cols() != v.size()");

  MatrixXd P_dot
      = A * m_P + m_P * A.transpose() + L * m_system->get_Q() * L.transpose();
  //- m_P * H.transpose()
  //     * (M * m_system->get_R() * M.transpose()).inverse() * H * m_P;

  // std::cout << "P_dot = " << P_dot.diagonal().transpose() << std::endl;

  MatrixXd P = m_P + P_dot * m_dt;

  // if (m_first)
  //   m_first = false;
  // else
  //   P += P_dot * m_dt; // P_k^-

  // m_x = x;
  // m_P = P;

  // step 2: update (equ. 13.33)
  MatrixXd K = MatrixXd::Zero(m_system->get_xdim(), m_system->get_ydim());

  VectorXd err_y = y - m_system->h(x, VectorXd::Zero(m_system->get_vdim()));

  if (m_system->get_const_dim() > 0 && !m_linearized_constraints) {
    // solve the constrained optimization problem
    m_constraints_system->update(x, u, err_y, P, H, M);

    // m_constraints_system->verify_diff(
    //     VectorXd::Random(m_system->get_const_dim()) * 10);

    bool converged = false;
    // if (!m_lambda.isZero())
    //   std::tie(m_lambda, converged) = m_solver_newtons->solve(m_lambda);

    if (!converged) {
      // std::cout << "IPOPT" << std::endl;

      // homotopy method
      // m_lambda = m_solver_homotopy->solve(1e3, 50, 1e-3);

      // IPOPT method
      // try 10 times
      // m_lambda_cost->set_target(m_lambda);

      // if (!m_lambda.isZero())
      //   m_lambda_var->set_bounds(m_lambda, 0.5);

      for (int i = 0; i < 10; i++) {
        math::VectorXd lambda_guess;
        if (i == 0 && !m_lambda.isZero())
          lambda_guess = m_lambda;
        else
          lambda_guess = m_lambda
                         //  * (1 - std::rand() % 100 * 0.005);
                         + VectorXd::Random(m_system->get_const_dim())
                               * 0.02; // 0.05 / 0.1
        m_lambda_var->set_variables(lambda_guess);
        math::VectorXd scale
            = VectorXd::Ones(m_system->get_const_dim()) * (i + 3) / 3.0;
        scale[0] = 1.0;
        scale[1] = 1.0;
        m_lambda_constraint->set_scale(scale);

        m_solver->solve(*m_nlp);
        if (m_solver->get_return_status() == IpoptSolver::SUCCEEDED) {
          break;
        }
      }

      // if (m_lambda.size() != 0) {
      //   m_lambda_var->set_variables(m_lambda);
      // }
      // m_solver->solve(*m_nlp);

      if (m_solver->get_return_status() == IpoptSolver::SUCCEEDED) {
        m_lambda = m_nlp->get_variable_values();
      }
      else {
        CROBOT_ASSERT_WARNING(false, "IpoptSolver failed in EKF!");
        return false;
      }
    }

    // std::cout << "lambda range: (" << m_lambda.minCoeff() << ", "
    //           << m_lambda.maxCoeff() << ")" << std::endl;
    K = m_constraints_system->get_gain(m_lambda);

    // std::cout << "K: " << std::endl << K << std::endl;
  }
  else {
    // solve the unconstrained optimization problem
    K = P * H.transpose()
        * (H * P * H.transpose() + M * m_system->get_R() * M.transpose())
              .inverse();

    // K = P * H.transpose()
    //     * math::calculate_jacb_inv(
    //         H * P * H.transpose() + M * m_system->get_R() * M.transpose());
  }

  math::VectorXd dx = K * err_y;

  // std::cout << "dx = " << dx.transpose() << std::endl;

  CROBOT_ASSERT_ERROR(
      dx.maxCoeff() < 1.0 || dx.minCoeff() > -1.0, "dx is too large");

  m_x = x + dx; // x_k^+

  MatrixXd I_KH = MatrixXd::Identity(P.rows(), P.cols()) - K * H;

  // std::cout << "I_KH * P * I_KH = "
  //           << (I_KH * P * I_KH.transpose()).diagonal().transpose()
  //           << std::endl;

  m_P = I_KH * P * I_KH.transpose()
        + K * M * m_system->get_R() * M.transpose() * K.transpose();

  // std::cout << "P = " << m_P.diagonal().transpose() << std::endl;

  // if (m_linearized_constraints) {
  //   // project x and P to the constraint manifold
  //   math::MatrixXd A;
  //   math::VectorXd b;
  //   std::tie(A, b) = m_system->get_linear_constraints(m_x);

  //   // MatrixXd Wk_inv = MatrixXd::Identity(m_P.rows(), m_P.cols());
  //   MatrixXd Wk_inv = m_P;
  //   MatrixXd project_mat
  //       = Wk_inv * A.transpose() * (A * Wk_inv * A.transpose()).inverse();

  //   // MatrixXd project_mat
  //   //     = Wk_inv * A.transpose()
  //   //       * math::calculate_jacb_inv(A * Wk_inv * A.transpose());

  //   m_x = m_x - project_mat * (A * m_x - b);
  //   m_P = (MatrixXd::Identity(m_P.rows(), m_P.cols()) - project_mat * A) *
  //   m_P;
  // }

  // m_lambda_var->set_bounds(m_lambda, 1.0);

  return true;
}

//==============================================================================
void KalmanFilterSolver::reset() {
  math::MatrixXd P_reset = m_system->get_P0_reset();
  for (int i = 0; i < P_reset.rows(); i++) {
    if (P_reset(i, i) < 0) { // do not reset
      P_reset(i, i) = m_P(i, i);
    }
  }
  m_P = P_reset;
  m_lambda.setZero();
  // m_lambda_var->set_bounds(m_lambda, 5.0);
  m_x.segment<6>(7) = math::VectorXd::Zero(6);
}
