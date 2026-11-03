#include <iomanip>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/composite.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
Composite::Composite(const std::string& name, bool is_cost)
  : Component(0, name) {
  m_is_cost = is_cost;
}

//==============================================================================
void Composite::add_component(const Component::Ptr& c) {
  // at this point the number of rows must be specified.
  assert(c->get_rows() != kSpecifyLater);

  m_components.push_back(c);

  if (m_is_cost)
    set_rows(1);
  else
    set_rows(get_rows() + c->get_rows());
}

//==============================================================================
void Composite::clear_components() {
  m_components.clear();
  set_rows(0);
}

//==============================================================================
const Component::Ptr Composite::get_component(std::string name) const {
  for (const auto& c : m_components)
    if (c->get_name() == name)
      return c;

  assert(false); // component with name doesn't exist, abort program
  return Component::Ptr();
}

//==============================================================================
math::VectorXd Composite::get_values() const {
  math::VectorXd g_all = math::VectorXd::Zero(get_rows());

  int row = 0;
  for (const auto& c : m_components) {
    int n_rows       = c->get_rows();
    math::VectorXd g = c->get_values();
    g_all.middleRows(row, n_rows) += g;

    if (!m_is_cost)
      row += n_rows;
  }
  return g_all;
}

//==============================================================================
void Composite::set_variables(const math::VectorXd& x) {
  int row = 0;
  for (auto& c : m_components) {
    int n_rows = c->get_rows();
    c->set_variables(x.middleRows(row, n_rows));
    row += n_rows;
  }
}

//==============================================================================
int Composite::get_rows() const {
  int rows = 0;
  for (const Component::Ptr& comp : m_components) {
    rows += comp->get_rows();
  }
  return rows;
}

//==============================================================================
math::SparseJacobian Composite::get_jacobian() const {
  // set number of variables only the first time this function is
  // called, since number doesn't change during the optimization. Improves
  // efficiency.
  if (m_num_var == -1)
    m_num_var = m_components.empty()
                    ? 0
                    : m_components.front()->get_jacobian().cols();

  math::SparseJacobian jacobian(get_rows(), m_num_var);

  if (m_num_var == 0)
    return jacobian;

  int row = 0;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& c : m_components) {
    const math::SparseJacobian& jac = c->get_jacobian();
    triplet_list.reserve(triplet_list.size() + jac.nonZeros());

    for (int k = 0; k < jac.outerSize(); ++k)
      for (math::SparseJacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.push_back(
            Eigen::Triplet<double>(row + it.row(), it.col(), it.value()));

    if (!m_is_cost)
      row += c->get_rows();
  }

  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

//==============================================================================
VecBound Composite::get_bounds() const {
  VecBound bounds;
  for (const auto& c : m_components) {
    VecBound b = c->get_bounds();
    bounds.insert(bounds.end(), b.begin(), b.end());
  }
  return bounds;
}

//==============================================================================
const Composite::ComponentVec Composite::get_components() const {
  return m_components;
}

//==============================================================================
void Composite::print_all() const {
  // starting index
  int index = 0;
  // tolerance when printing out constraint/bound violation.
  double tol = 0.001;

  std::cout << get_name() << ":\n";
  for (const auto& c : m_components) {
    CROBOT_ASSERT_ERROR(c != nullptr);
    std::cout << "   "; // indent components
    c->show_info(tol, index);
  }
  std::cout << std::endl;
}
