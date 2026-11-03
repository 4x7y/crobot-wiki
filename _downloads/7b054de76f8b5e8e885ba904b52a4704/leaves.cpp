#include <iomanip>
#include <iostream>

#include "crobot/math/matrix.hpp"
#include "crobot/optimization/constraint_set.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/variable_set.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
VariableSet::VariableSet(int n_var, const std::string& name)
  : Component(n_var, name) {
}

//==============================================================================
ConstraintSet::ConstraintSet(
    int row_count,
    const std::string& name,
    const std::vector<std::string>& var_names)
  : Component(row_count, name), m_varset_names(var_names) {
}

//==============================================================================
math::SparseJacobian ConstraintSet::get_jacobian() const {
  math::SparseJacobian jacobian(get_rows(), m_variables->get_rows());

  int col = 0;
  math::SparseJacobian jac;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& vars : m_variables->get_components()) {
    int n = vars->get_rows();
    jac.resize(get_rows(), n);

    fill_jacobian_block(vars, jac);
    // reserve space for the new elements to reduce memory allocation
    triplet_list.reserve(triplet_list.size() + jac.nonZeros());

    // create triplets for the derivative at the correct position in the overall
    // Jacobian
    for (int k = 0; k < jac.outerSize(); ++k)
      for (math::SparseJacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.push_back(
            Eigen::Triplet<double>(it.row(), col + it.col(), it.value()));
    col += n;
  }

  // transform triplet vector into sparse matrix
  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

//==============================================================================
math::SparseJacobian ConstraintSet::get_jacobian_finite_diff() const {
  math::SparseJacobian jacobian(get_rows(), m_variables->get_rows());
  int col = 0;
  math::SparseJacobian jac;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& vars : m_variables->get_components()) {
    int n = vars->get_rows();
    jac.resize(get_rows(), n);

    fill_jacobian_block_finite_diff(vars, jac);
    // reserve space for the new elements to reduce memory allocation
    triplet_list.reserve(triplet_list.size() + jac.nonZeros());

    // create triplets for the derivative at the correct position in the overall
    // Jacobian
    for (int k = 0; k < jac.outerSize(); ++k)
      for (math::SparseJacobian::InnerIterator it(jac, k); it; ++it)
        triplet_list.push_back(
            Eigen::Triplet<double>(it.row(), col + it.col(), it.value()));
    col += n;
  }

  // transform triplet vector into sparse matrix
  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

//==============================================================================
void ConstraintSet::fill_jacobian_block(
    Component::Ptr varset, math::SparseJacobian& jac_block) const {
  fill_jacobian_block_finite_diff(varset, jac_block);
}

//==============================================================================
void ConstraintSet::fill_jacobian_block_finite_diff(
    Component::Ptr varset, math::SparseJacobian& jac_block, double eps) const {

  auto addr_iter = m_varset_addrs.find(varset->get_name());
  if (addr_iter != m_varset_addrs.end()) {
    std::pair<int, int> addr = addr_iter->second;
    math::VectorXd x         = get_dependent_variable_values();
    for (int j = 0; j < addr.second; j++) {
      math::VectorXd x_l = x;
      x_l(j + addr.first) -= eps;
      math::VectorXd conval_l = get_values(x_l);

      math::VectorXd x_r = x;
      x_r(j + addr.first) += eps;
      math::VectorXd conval_r = get_values(x_r);

      math::VectorXd jac_col = (conval_r - conval_l) / (eps * 2.);
      for (int i = 0; i < jac_col.size(); i++) {
        jac_block.coeffRef(i, j) = jac_col(i);
      }
    }
  }
}

//==============================================================================
void ConstraintSet::link_with_variables(const VariablesPtr& x) {
  m_variables = x;

  m_varset_addrs.clear();
  int addr = 0;
  for (const std::string& varname : m_varset_names) {
    int nrows               = m_variables->get_component(varname)->get_rows();
    m_varset_addrs[varname] = std::make_pair<>(addr, nrows);
    addr += nrows;
  }
}

//==============================================================================
math::VectorXd ConstraintSet::get_dependent_variable_values() const {
  int addr = 0;
  std::vector<Component::Ptr> varsets;
  for (const std::string& varset_name : m_varset_names) {
    Component::Ptr varset = m_variables->get_component(varset_name);
    addr += varset->get_rows();
    varsets.push_back(varset);
  }

  math::VectorXd x(addr);
  addr = 0;
  for (Component::Ptr varset : varsets) {
    x.segment(addr, varset->get_rows()) = varset->get_values();
    addr += varset->get_rows();
  }

  return x;
}

//==============================================================================
math::VectorXd ConstraintSet::get_values() const {
  math::VectorXd x = get_dependent_variable_values();
  return get_values(x);
}

//==============================================================================
void ConstraintSet::add_dependent_varset(const std::string& varname) {
  m_varset_names.push_back(varname);
}

//==============================================================================
CostTerm::CostTerm(
    const std::string& name, const std::vector<std::string>& variables)
  : ConstraintSet(1, name, variables) {
}

//==============================================================================
math::VectorXd CostTerm::get_values(const math::VectorXd& x) const {
  math::VectorXd cost(1);
  cost(0) = get_cost(x);
  return cost;
}

//==============================================================================
VecBound CostTerm::get_bounds() const {
  return VecBound(get_rows(), NoBound);
}

//==============================================================================
void CostTerm::show_info(double /* tol */, int& index) const {
  // only one scalar cost value
  double cost = get_values()(0);

  std::cout.precision(2);
  std::cout << std::fixed << std::left << std::setw(30) << get_name()
            << std::right << std::setw(4) << get_rows() << std::setw(9) << index
            << std::setfill('.') << std::setw(7) << index + get_rows() - 1
            << std::setfill(' ') << std::setw(12) << cost << std::endl;
}
