#pragma once

#include "crobot/ekf/kalman_filter_system.hpp"
#include "crobot/ekf/polynomial_system.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/optimization.hpp"

namespace crobot::ekf {

class KFConstrainedSystem : public PolynomialSystem {
public:
  using Ptr = std::shared_ptr<KFConstrainedSystem>;

  KFConstrainedSystem(KalmanFilterSystem::ConstPtr kf_system);

  math::VectorXd f(const math::VectorXd& x) const override;
  math::MatrixXd dfdx(const math::VectorXd& x) const override;
  math::MatrixXd get_gain(const math::VectorXd& lambda) const;

  void update(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& err_y,
      const math::MatrixXd& P,
      const math::MatrixXd& H,
      const math::MatrixXd& M);

private:
  std::tuple<math::MatrixXd, math::VectorXd, double> get_J_m_eps(
      const math::VectorXd& lambda) const;
  KalmanFilterSystem::ConstPtr m_kf_system;

  bool updated;

  math::VectorXd m_x;
  math::VectorXd m_u;
  math::VectorXd m_err_y;
  math::MatrixXd m_P;
  math::MatrixXd m_H;
  math::MatrixXd m_R;

  std::vector<math::MatrixXd> m_constraints_A;
  std::vector<math::VectorXd> m_constraints_b;
  std::vector<double> m_constraints_c;
};

class LambdaVariable : public optimization::VariableSet {
public:
  LambdaVariable(int dim);
  void set_variables(const math::VectorXd& value) override;
  math::VectorXd get_values() const override;
  optimization::VecBound get_bounds() const override;
  void set_bounds(const math::VectorXd& center, double eps);

private:
  math::VectorXd m_value;
  optimization::VecBound bounds;
};

class LambdaConstraint : public optimization::ConstraintSet {
public:
  LambdaConstraint(KFConstrainedSystem::Ptr polysystem, math::VectorXd eps);
  math::VectorXd get_values(const math::VectorXd& x) const override;
  optimization::VecBound get_bounds() const override;
  void set_scale(const math::VectorXd& scale);
  void fill_jacobian_block(
      Component::Ptr varsets, math::SparseJacobian& jac_block) const override;

private:
  KFConstrainedSystem::Ptr m_polysystem;
  math::VectorXd m_eps;
  math::VectorXd m_scale;
};

class LambdaCost : public optimization::CostTerm {
public:
  LambdaCost(int dim);

  double get_cost(const math::VectorXd& x) const override;

  // Documentation inherited
  void fill_jacobian_block(
      Component::Ptr var_set, math::SparseJacobian& jac) const override;

  void set_target(const math::VectorXd& target);

private:
  math::VectorXd m_target;
};

/**
 * @brief Description of a continuous-time dynamic system with discrete-time
 * measurements for hybrid extended Kalman filter. (P405)
 */
class KalmanFilterSolver {
public:
  using Ptr = std::shared_ptr<KalmanFilterSolver>;

  KalmanFilterSolver(
      KalmanFilterSystem::Ptr system,
      double dt,
      double max_cpu_time         = 0.1,
      bool linearized_constraints = false);

  /**
   * @brief Update the state vector and covariance matrix using the Kalman
   * filter equations.
   *
   * @param y Measurement vector at time k
   * @param u Input vector at time k
   * @return False for non-finite data, singular innovation, or failed
   * constraints; the state and covariance are unchanged. Invalid dimensions
   * throw. Q is a continuous-time noise spectral density; R is measurement
   * covariance.
   */
  bool update(const math::VectorXd& y, const math::VectorXd& u);

  /**
   * @brief Get the state vector at time k
   *
   * @return math::VectorXd
   */
  math::VectorXd get_state() const {
    return m_x;
  }

  math::VectorXd get_variance() const {
    return m_P.diagonal();
  }

  math::MatrixXd get_covariance() const {
    return m_P;
  }

  void reset();

private:
  void init_constraints_solver(double max_cpu_time);

  KalmanFilterSystem::Ptr m_system;
  double m_dt;

  math::VectorXd m_x; // state vector at time k
  math::MatrixXd m_P; // state covariance matrix at time k

  std::shared_ptr<optimization::Problem> m_nlp;
  std::shared_ptr<optimization::IpoptSolver> m_solver;

  KFConstrainedSystem::Ptr m_constraints_system;

  math::VectorXd m_lambda;

  std::shared_ptr<ekf::LambdaVariable> m_lambda_var;
  std::shared_ptr<ekf::LambdaConstraint> m_lambda_constraint;

  bool m_linearized_constraints;
};

} // namespace crobot::ekf
