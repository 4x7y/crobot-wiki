#include <stdexcept>

#include "crobot/optimization/linear_program.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
// File-local helpers
//==============================================================================

namespace {

int require_nonnegative_size(int size, const std::string& what) {
  if (size < 0) {
    throw std::invalid_argument(what + " must be non-negative");
  }

  return size;
}

VecBound make_unbounded_bounds(int size) {
  return VecBound(size, NoBound);
}

void validate_vector_size(
    const math::VectorXd& values, int expected_size, const std::string& what) {
  if (values.size() != expected_size) {
    throw std::invalid_argument(
        what + " size mismatch: expected " + std::to_string(expected_size)
        + ", got " + std::to_string(values.size()));
  }
}

void validate_matrix_cols(
    const math::MatrixXd& matrix, int expected_cols, const std::string& what) {
  if (matrix.cols() != expected_cols) {
    throw std::invalid_argument(
        what + " column mismatch: expected " + std::to_string(expected_cols)
        + ", got " + std::to_string(matrix.cols()));
  }
}

VecBound normalize_bounds(const VecBound& bounds, int size) {
  if (bounds.empty()) {
    return make_unbounded_bounds(size);
  }

  if (static_cast<int>(bounds.size()) != size) {
    throw std::invalid_argument(
        "bounds size mismatch: expected " + std::to_string(size) + ", got "
        + std::to_string(bounds.size()));
  }

  return bounds;
}

} // namespace

//==============================================================================
// detail::LinearVariableSet
//==============================================================================

detail::LinearVariableSet::LinearVariableSet(
    const std::string& name,
    int num_variables,
    const VecBound& bounds,
    const math::VectorXd& initial_guess)
  : VariableSet(
      require_nonnegative_size(num_variables, "number of variables"), name),
    m_values(math::VectorXd::Zero(num_variables)),
    m_bounds(normalize_bounds(bounds, num_variables)) {
  if (initial_guess.size() > 0) {
    validate_vector_size(initial_guess, num_variables, "initial guess");
    m_values = initial_guess;
  }
}

//==============================================================================
void detail::LinearVariableSet::set_variables(const math::VectorXd& x) {
  validate_vector_size(x, get_rows(), "variable vector");
  m_values = x;
}

//==============================================================================
math::VectorXd detail::LinearVariableSet::get_values() const {
  return m_values;
}

//==============================================================================
VecBound detail::LinearVariableSet::get_bounds() const {
  return m_bounds;
}

//==============================================================================
// LinearCost
//==============================================================================

LinearCost::LinearCost(
    const std::string& name,
    const std::string& variable_name,
    const math::VectorXd& coefficients,
    double constant)
  : CostTerm(name, {variable_name}),
    m_coefficients(coefficients),
    m_constant(constant),
    m_variable_name(variable_name) {
}

//==============================================================================
double LinearCost::get_cost(const math::VectorXd& x) const {
  validate_vector_size(x, m_coefficients.size(), "cost variables");
  return m_coefficients.dot(x) + m_constant;
}

//==============================================================================
void LinearCost::fill_jacobian_block(
    Component::Ptr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() != m_variable_name) {
    return;
  }

  if (varset->get_rows() != m_coefficients.size()) {
    throw std::invalid_argument(
        "cost variable dimension mismatch for '" + m_variable_name + "'");
  }

  for (int column = 0; column < m_coefficients.size(); ++column) {
    if (m_coefficients(column) != 0.0) {
      jac_block.coeffRef(0, column) = m_coefficients(column);
    }
  }
}

//==============================================================================
const math::VectorXd& LinearCost::get_coefficients() const {
  return m_coefficients;
}

//==============================================================================
double LinearCost::get_constant() const {
  return m_constant;
}

//==============================================================================
// LinearConstraint
//==============================================================================

LinearConstraint::LinearConstraint(
    const std::string& name,
    const std::string& variable_name,
    const math::MatrixXd& coefficients,
    const VecBound& bounds)
  : ConstraintSet(coefficients.rows(), name, {variable_name}),
    m_coefficients(coefficients),
    m_bounds(normalize_bounds(bounds, coefficients.rows())),
    m_variable_name(variable_name) {
}

//==============================================================================
LinearConstraint::LinearConstraint(
    const std::string& name,
    const std::string& variable_name,
    const math::MatrixXd& coefficients,
    const math::VectorXd& lower_bounds,
    const math::VectorXd& upper_bounds)
  : LinearConstraint(
      name,
      variable_name,
      coefficients,
      make_bounds(lower_bounds, upper_bounds)) {
  validate_vector_size(
      lower_bounds, coefficients.rows(), "constraint lower bounds");
  validate_vector_size(
      upper_bounds, coefficients.rows(), "constraint upper bounds");
}

//==============================================================================
math::VectorXd LinearConstraint::get_values(const math::VectorXd& x) const {
  validate_matrix_cols(
      m_coefficients, x.size(), "constraint coefficient matrix");
  return m_coefficients * x;
}

//==============================================================================
VecBound LinearConstraint::get_bounds() const {
  return m_bounds;
}

//==============================================================================
void LinearConstraint::fill_jacobian_block(
    Component::Ptr varset, math::SparseJacobian& jac_block) const {
  if (varset->get_name() != m_variable_name) {
    return;
  }

  if (varset->get_rows() != m_coefficients.cols()) {
    throw std::invalid_argument(
        "constraint variable dimension mismatch for '" + m_variable_name + "'");
  }

  for (int row = 0; row < m_coefficients.rows(); ++row) {
    for (int column = 0; column < m_coefficients.cols(); ++column) {
      if (m_coefficients(row, column) != 0.0) {
        jac_block.coeffRef(row, column) = m_coefficients(row, column);
      }
    }
  }
}

//==============================================================================
const math::MatrixXd& LinearConstraint::get_coefficients() const {
  return m_coefficients;
}

//==============================================================================
VecBound LinearConstraint::make_bounds(
    const math::VectorXd& lower_bounds, const math::VectorXd& upper_bounds) {
  if (lower_bounds.size() != upper_bounds.size()) {
    throw std::invalid_argument(
        "constraint bound vectors must have the same size");
  }

  VecBound bounds(lower_bounds.size());
  for (int row = 0; row < lower_bounds.size(); ++row) {
    bounds.at(row) = Bounds(lower_bounds(row), upper_bounds(row));
  }

  return bounds;
}

//==============================================================================
// LinearProgram
//==============================================================================

LinearProgram::LinearProgram(
    int num_variables,
    const VecBound& variable_bounds,
    const math::VectorXd& initial_guess,
    const std::string& variable_name)
  : m_variable_name(variable_name),
    m_variables(std::make_shared<detail::LinearVariableSet>(
        variable_name, num_variables, variable_bounds, initial_guess)) {
  add_variable_set(m_variables);
}

//==============================================================================
LinearProgram::LinearProgram(
    const math::VectorXd& objective,
    const math::MatrixXd& constraint_matrix,
    const VecBound& constraint_bounds,
    const VecBound& variable_bounds,
    const math::VectorXd& initial_guess,
    const std::string& variable_name,
    const std::string& cost_name,
    const std::string& constraint_name)
  : LinearProgram(
      objective.size(), variable_bounds, initial_guess, variable_name) {
  set_linear_cost(cost_name, objective);

  if (constraint_matrix.rows() > 0) {
    add_linear_constraint_set(
        constraint_name, constraint_matrix, constraint_bounds);
  }
}

//==============================================================================
LinearCost::Ptr LinearProgram::set_linear_cost(
    const std::string& name,
    const math::VectorXd& coefficients,
    double constant) {
  if (m_cost != nullptr) {
    throw std::logic_error("linear program already has a cost term");
  }

  validate_vector_size(
      coefficients, m_variables->get_rows(), "objective coefficients");
  m_cost = std::make_shared<LinearCost>(
      name, m_variable_name, coefficients, constant);
  add_cost_set(m_cost);
  return m_cost;
}

//==============================================================================
LinearConstraint::Ptr LinearProgram::add_linear_constraint_set(
    const std::string& name,
    const math::MatrixXd& coefficients,
    const VecBound& bounds) {
  validate_matrix_cols(
      coefficients, m_variables->get_rows(), "constraint coefficient matrix");
  auto constraint = std::make_shared<LinearConstraint>(
      name, m_variable_name, coefficients, bounds);
  add_constraint_set(constraint);
  return constraint;
}

//==============================================================================
LinearConstraint::Ptr LinearProgram::add_linear_constraint_set(
    const std::string& name,
    const math::MatrixXd& coefficients,
    const math::VectorXd& lower_bounds,
    const math::VectorXd& upper_bounds) {
  validate_matrix_cols(
      coefficients, m_variables->get_rows(), "constraint coefficient matrix");
  auto constraint = std::make_shared<LinearConstraint>(
      name, m_variable_name, coefficients, lower_bounds, upper_bounds);
  add_constraint_set(constraint);
  return constraint;
}

//==============================================================================
void LinearProgram::set_initial_guess(const math::VectorXd& initial_guess) {
  m_variables->set_variables(initial_guess);
}

//==============================================================================
const std::string& LinearProgram::get_variable_name() const {
  return m_variable_name;
}

//==============================================================================
int LinearProgram::get_num_decision_variables() const {
  return m_variables->get_rows();
}