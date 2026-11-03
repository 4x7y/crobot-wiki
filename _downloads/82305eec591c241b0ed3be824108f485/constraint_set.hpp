#pragma once

#include "crobot/optimization/composite.hpp"

namespace crobot::optimization {

/**
 * @brief A container holding a set of related constraints.
 *
 * This container holds constraints representing a single concept, e.g.
 * @c n constraints keeping a foot inside its range of motion. Each of the
 * @c n rows is given by:
 * lower_bound < g(x) < upper_bound
 *
 * These constraint sets are later then stitched together to form the overall
 * problem.
 */
class ConstraintSet : public Component {
public:
  using Ptr          = std::shared_ptr<ConstraintSet>;
  using VariablesPtr = Composite::Ptr;

  /**
   * @brief Creates constraints on the variables @c x.
   * @param n_constraints  The number of constraints.
   * @param name  What these constraints represent.
   * @param variables Name of the dependent variables.
   */
  ConstraintSet(
      int n_constraints,
      const std::string& name,
      const std::vector<std::string>& variables = {});

  /**
   * @brief Destroy the ConstraintSet object.
   */
  virtual ~ConstraintSet() = default;

  /**
   * @brief Connects the constraint with the optimization variables.
   * @param x  A pointer to the current values of the optimization variables.
   *
   * The optimization variable values are necessary for calculating constraint
   * violations and Jacobians.
   */
  void link_with_variables(const VariablesPtr& x);

  /**
   * @brief Get the dependent variables and concatenate their values together.
   */
  virtual math::VectorXd get_dependent_variable_values() const;

  // Documentation inherited.
  math::VectorXd get_values() const override final;

  /**
   * @brief The matrix of derivatives for these constraints and variables.
   *
   * Assuming @c n constraints and @c m variables, the returned Jacobian
   * has dimensions n x m. Every row represents the derivatives of a single
   * constraint, whereas every column refers to a single optimization variable.
   *
   * This function only combines the user-defined jacobians from
   * fill_jacobian_block().
   */
  math::SparseJacobian get_jacobian() const override;

  /**
   * @brief Returns the matrix of derivatives for these constraints and
   * variables using finite differentiation method.
   *
   * Assuming @c n constraints and @c m variables, the returned Jacobian
   * has dimensions n x m. Every row represents the derivatives of a single
   * constraint, whereas every column refers to a single optimization variable.
   *
   * This function only combines the user-defined jacobians from
   * fill_jacobian_block_finite_diff().
   */
  virtual math::SparseJacobian get_jacobian_finite_diff() const;

  /**
   * @brief Set individual Jacobians corresponding to each decision variable
   * set.
   * @param varset  Set of variables the current Jacobian block belongs to.
   * @param jac_block  Columns of the overall Jacobian affected by varset.
   *
   * A convenience function so the user does not have to worry about the
   * ordering of variable sets. All that is required is that the user knows
   * the internal ordering of variables in each individual set and provides
   * the Jacobian of the constraints w.r.t. this set (starting at column 0).
   * get_jacobian() then inserts these columns at the correct position in the
   * overall Jacobian.
   *
   * If the constraint doen't depend on a @c varset, this function should
   * simply do nothing.
   *
   * Attention: @c jac_bock is a sparse matrix, and must always have the same
   * sparsity pattern. Therefore, it's better not to build a dense matrix and
   * call .sparseView(), because if some entries happen to be zero for some
   * iteration, that changes the sparsity pattern. A more robust way is to
   * directly set as follows (which can also be set =0.0 without erros):
   * jac_block.coeffRef(1, 3) = ...
   */
  virtual void fill_jacobian_block(
      Component::Ptr varset, math::SparseJacobian& jac_block) const;

protected:
  /**
   * @brief Read access to the value of the optimization variables.
   *
   * This must be used to formulate the constraint violation and Jacobian.
   */
  const VariablesPtr get_variables() const {
    return m_variables;
  };

  /**
   * @brief Adds a dependent variable.
   */
  void add_dependent_varset(const std::string& varname);

  /**
   * @brief Get the constraint values given the values of its dependent
   * variables.
   */
  virtual math::VectorXd get_values(const math::VectorXd& variables) const = 0;

  /**
   * @brief Set individual Jacobians corresponding to each decision variable
   * set using numerical differentiation.
   *
   * @param varset Set of variables the current Jacobian block belongs to.
   * @param jac_block Columns of the overall Jacobian affected by varset.
   * @param eps Epsilon for finite differentiation.
   */
  virtual void fill_jacobian_block_finite_diff(
      Component::Ptr varset,
      math::SparseJacobian& jac_block,
      double eps = 1e-6) const;

protected:
  std::vector<std::string> m_varset_names;

private:
  VariablesPtr m_variables;
  std::unordered_map<std::string, std::pair<int, int>> m_varset_addrs;

  // doesn't exist for constraints, generated run-time error when used
  void set_variables(const math::VectorXd& /* x */) final {
    assert(false);
  };
};

} // namespace crobot::optimization

