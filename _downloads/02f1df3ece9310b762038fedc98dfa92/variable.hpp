#pragma once

#include "crobot/optimization/variable_set.hpp"

namespace crobot::ik {

class Variable : public optimization::VariableSet {
public:
  using Ptr      = std::shared_ptr<Variable>;
  using ConstPtr = std::shared_ptr<const Variable>;

  Variable(int n_var, const std::string& name);

  /**
   * @brief Get the number of variables in this variable set.
   *
   * @return int Number of variables.
   */
  virtual int numvars_all() const;

  /**
   * @brief Get the number of fixed variables in this variable set.
   *
   * @return int Number of fixed variables.
   */
  virtual int numvars_fixed() const;

  /**
   * @brief Get the number of free variables in this variable set.
   *
   * @return int Number of free variables.
   */
  virtual int numvars_free() const;

  /**
   * @brief Set the values of all variables, including free variables and fixed
   * variables.
   *
   * @param value The values to set.
   */
  virtual void set_values_all(const math::VectorXd& value) = 0;

  /**
   * @brief Get the values of all variables, including free variables and fixed
   * variables.
   *
   * @return math::VectorXd The values of all variables.
   */
  virtual math::VectorXd get_values_all() const = 0;

  /**
   * @brief Get the bounds of all variables, including free variables and fixed
   * variables.
   *
   * @return optimization::VecBound The bounds of all variables.
   */
  virtual optimization::VecBound get_bounds_all() const = 0;

  /**
   * @brief Fix a group of variables to certain values.
   *
   * @param range The index range of variables to fix.
   * @param value The values to fix the variables to.
   */
  virtual void fix_vars(
      const std::pair<int, int>& range, const math::VectorXd& value);

  /**
   * @brief Get the indices of free variables.
   *
   * @return const std::vector<int>& The indices of free variables.
   */
  std::vector<int> get_freevar_indices() const;

  /**
   * @brief Map a variable index to the corrsponding index of free variable
   * vector. Returns -1 if the index is not pointing to a free variable.
   *
   * @param index The index of variable.
   * @return int The index in the free variable vector.
   */
  int map_index(int index) const;

protected:
private:
  // Documentation inherited
  math::VectorXd get_values() const override final;

  // Documentation inherited
  void set_variables(const math::VectorXd& x) override final;

  // Documentation inherited
  optimization::VecBound get_bounds() const override final;

  // Documentation inherited
  using optimization::Component::get_rows;
  using optimization::Component::set_rows;

  math::VectorXd m_vars;

  /// Index of free variables in a free varaible set. The index is -1 if the
  /// variable is fixed.
  math::VectorXi m_indices;
};

} // namespace crobot::ik

