#pragma once

#include <string>

#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/problem.hpp"

namespace crobot::optimization {

namespace detail {

class LinearVariableSet : public VariableSet {
public:
  using Ptr = std::shared_ptr<LinearVariableSet>;

  LinearVariableSet(
      const std::string& name,
      int num_variables,
      const VecBound& bounds              = {},
      const math::VectorXd& initial_guess = math::VectorXd());

  void set_variables(const math::VectorXd& x) override;
  math::VectorXd get_values() const override;
  VecBound get_bounds() const override;

private:
  math::VectorXd m_values;
  VecBound m_bounds;
};

} // namespace detail

//==============================================================================
class LinearCost : public CostTerm {
public:
  using Ptr = std::shared_ptr<LinearCost>;

  LinearCost(
      const std::string& name,
      const std::string& variable_name,
      const math::VectorXd& coefficients,
      double constant = 0.0);

  double get_cost(const math::VectorXd& x) const override;

  void fill_jacobian_block(
      Component::Ptr varset, math::SparseJacobian& jac_block) const override;

  const math::VectorXd& get_coefficients() const;
  double get_constant() const;

private:
  math::VectorXd m_coefficients;
  double m_constant;
  std::string m_variable_name;
};

//==============================================================================
class LinearConstraint : public ConstraintSet {
public:
  using Ptr = std::shared_ptr<LinearConstraint>;

  LinearConstraint(
      const std::string& name,
      const std::string& variable_name,
      const math::MatrixXd& coefficients,
      const VecBound& bounds);

  LinearConstraint(
      const std::string& name,
      const std::string& variable_name,
      const math::MatrixXd& coefficients,
      const math::VectorXd& lower_bounds,
      const math::VectorXd& upper_bounds);

  math::VectorXd get_values(const math::VectorXd& x) const override;
  VecBound get_bounds() const override;

  void fill_jacobian_block(
      Component::Ptr varset, math::SparseJacobian& jac_block) const override;

  const math::MatrixXd& get_coefficients() const;

private:
  static VecBound make_bounds(
      const math::VectorXd& lower_bounds, const math::VectorXd& upper_bounds);

  math::MatrixXd m_coefficients;
  VecBound m_bounds;
  std::string m_variable_name;
};

//==============================================================================
class LinearProgram : public Problem {
public:
  LinearProgram(
      int num_variables,
      const VecBound& variable_bounds     = {},
      const math::VectorXd& initial_guess = math::VectorXd(),
      const std::string& variable_name    = "x");

  LinearProgram(
      const math::VectorXd& objective,
      const math::MatrixXd& constraint_matrix = math::MatrixXd(0, 0),
      const VecBound& constraint_bounds       = {},
      const VecBound& variable_bounds         = {},
      const math::VectorXd& initial_guess     = math::VectorXd(),
      const std::string& variable_name        = "x",
      const std::string& cost_name            = "linear_cost",
      const std::string& constraint_name      = "linear_constraint");

  LinearCost::Ptr set_linear_cost(
      const std::string& name,
      const math::VectorXd& coefficients,
      double constant = 0.0);

  LinearConstraint::Ptr add_linear_constraint_set(
      const std::string& name,
      const math::MatrixXd& coefficients,
      const VecBound& bounds);

  LinearConstraint::Ptr add_linear_constraint_set(
      const std::string& name,
      const math::MatrixXd& coefficients,
      const math::VectorXd& lower_bounds,
      const math::VectorXd& upper_bounds);

  void set_initial_guess(const math::VectorXd& initial_guess);
  const std::string& get_variable_name() const;
  int get_num_decision_variables() const;

private:
  std::string m_variable_name;
  LinearCost::Ptr m_cost;
  detail::LinearVariableSet::Ptr m_variables;
};

} // namespace crobot::optimization