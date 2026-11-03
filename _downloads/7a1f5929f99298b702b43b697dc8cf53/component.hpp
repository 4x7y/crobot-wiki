#pragma once

#include <string>

#include "crobot/math/matrix.hpp"
#include "crobot/optimization/bounds.hpp"

namespace crobot::optimization {

/**
 * @brief Interface representing either Variable, Cost or Constraint.
 *
 * Variables, costs and constraints can all be fit into the same interface
 * (Component). For example, each has a "value", which is either the actual
 * value of the variables, the constraint value g or the cost.
 * This representation provides one common interface
 * ("smallest common denominator") that can be contain either individual
 * variables/costs/constraints or a Composite of these. This pattern
 * takes care of stacking variables, ordering Jacobians and providing bounds
 * for the complete problem without duplicating code. For more information on
 * the composite pattern visit
 * https://sourcemaking.com/design_patterns/composite
 */
class Component {
public:
  using Ptr = std::shared_ptr<Component>;

  /**
   * @brief  Creates a component.
   * @param  num_rows  The number of rows of this components.
   * @param  name  The identifier for this component.
   *
   * The number of rows @c num_rows can represent either
   * @li number of variables in this variables set
   * @li number of constraints in this constraint set
   * @li 1 if this component represents a Cost.
   */
  Component(int num_rows, const std::string& name);
  virtual ~Component() = default;

  /**
   * @brief  Returns the "values" of whatever this component represents.
   *
   * @li For Variable this represents the actual optimization values.
   * @li For Constraint this represents the constraint value g.
   * @li For Cost this represents the cost value.
   */
  virtual math::VectorXd get_values() const = 0;

  /**
   * @brief  Returns the "bounds" of this component.
   *
   * @li For Variable these are the upper and lower variable bound.
   * @li For Constraint this represents the constraint bounds.
   * @li For Cost these done't exists (set to infinity).
   */
  virtual VecBound get_bounds() const = 0;

  /**
   * @brief  Sets the optimization variables from an Eigen vector.
   *
   * This is only done for Variable, where these are set from the current
   * values of the @ref solvers.-
   */
  virtual void set_variables(const math::VectorXd& x) = 0;

  /**
   * @brief  Returns derivatives of each row w.r.t. the variables
   *
   * @li For Constraint this is a matrix with one row per constraint.
   * @li For a Cost this is a row vector (gradient transpose).
   * @li Not sensible for Variable.
   */
  virtual math::SparseJacobian get_jacobian() const = 0;

  /**
   * @brief Sets the number of rows of this component.
   *
   * @attention This should correctly be done through constructor call, only
   * delay this by using @c kSpecifyLater if you have good reason.
   */
  void set_rows(int num_rows);

  /**
   * @brief Returns the number of rows of this component.
   */
  virtual int get_rows() const;

  /**
   * @brief Returns the name (id) of this component.
   */
  std::string get_name() const;

  /**
   * @brief Prints the relevant information (name, rows, values) of this
   * component.
   * @param tolerance  When to flag constraint/bound violation.
   * @param index_start  Of this specific variables-, constraint- or cost set.
   */
  virtual void show_info(double tolerance, int& index_start) const;

  static const int kSpecifyLater = -1;

private:
  int m_num_rows = kSpecifyLater;
  std::string m_name;
};

} // namespace crobot::optimization

