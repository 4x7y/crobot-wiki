#pragma once

#include <memory>
#include <vector>

#include "crobot/optimization/component.hpp"

namespace crobot::optimization {

/**
 * @brief A collection of components which is treated as another Component.
 *
 * This class follows the Component interface as well, but doesn't actually
 * do any evaluation, but only stitches together the results of the
 * components it is holding. This is where multiple sets of variables,
 * constraints or costs are ordered and combined.
 *
 * See Component and Composite Pattern for more information.
 */
class Composite : public Component {
public:
  using Ptr          = std::shared_ptr<Composite>;
  using ComponentVec = std::vector<Component::Ptr>;

  /**
   * @brief  Creates a Composite holding either variables, costs or constraints.
   * @param  is_cost  True if this class holds cost terms, false for all others.
   *
   * Constraints and variables append individual constraint sets and Jacobian
   * rows below one another, whereas costs terms are all accumulated to a
   * scalar value/a single Jacobian row.
   */
  Composite(const std::string& name, bool is_cost);
  virtual ~Composite() = default;

  // Documentation inherited
  math::VectorXd get_values() const override;

  // Documentation inherited
  math::SparseJacobian get_jacobian() const override;

  // Documentation inherited
  VecBound get_bounds() const override;

  // Documentation inherited
  void set_variables(const math::VectorXd& x) override;

  // Documentation inherited
  int get_rows() const;

  void print_all() const;

  /**
   * @brief  Access generic component with the specified name.
   * @param  name  The name given to the component.
   * @return A generic pointer of that component.
   */
  const Component::Ptr get_component(std::string name) const;

  /**
   * @brief  Access type-casted component with the specified name.
   * @param  name  The name given to the component.
   * @tparam T  Type of component.
   * @return A type-casted pointer possibly providing addtional functionality.
   */
  template <typename T>
  std::shared_ptr<T> get_component(const std::string& name) const;

  /**
   * @brief Adds a component to this composite.
   */
  void add_component(const Component::Ptr&);

  /**
   * @brief Removes all component from this composite.
   */
  void clear_components();

  /**
   * @brief Returns read access to the components.
   */
  const ComponentVec get_components() const;

private:
  using Component::set_rows;

  /// Components that are part of this composite.
  ComponentVec m_components;

  /// Is this composite a cost term or not.
  bool m_is_cost;

  // The number of variables for costs/constraint composites (not set for
  // variables). Is initialized the first the get_jacobian() is called.
  mutable int m_num_var = -1;
};

// implementation of template functions
template <typename T>
std::shared_ptr<T> Composite::get_component(const std::string& name) const {
  Component::Ptr c = get_component(name);
  return std::dynamic_pointer_cast<T>(c);
}

} // namespace crobot::optimization

