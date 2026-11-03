#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/ik/constraint.hpp"
#include "crobot/ik/variable.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
Constraint::Constraint(
    int n_constraints,
    const std::string& name,
    const std::vector<std::string>& variables)
  : optimization::ConstraintSet(n_constraints, name, variables) {
}

//==============================================================================
math::VectorXd Constraint::get_dependent_variable_values() const {
  int addr = 0;
  std::vector<Variable::ConstPtr> varsets;
  for (const std::string& varset_name : m_varset_names) {
    Variable::Ptr varset = std::dynamic_pointer_cast<Variable>(
        get_variables()->get_component(varset_name));
    varsets.push_back(varset);
    addr += varset->numvars_all();
  }

  math::VectorXd x(addr);
  addr = 0;
  for (Variable::ConstPtr varset : varsets) {
    x.segment(addr, varset->numvars_all()) = varset->get_values_all();
    addr += varset->numvars_all();
  }
  return x;
}

//==============================================================================
math::SparseJacobian Constraint::get_jacobian() const {
  const int rows = get_rows();
  const int cols = get_variables()->get_rows();
  math::SparseJacobian jacobian(rows, cols);
  std::vector<Eigen::Triplet<double>> triplet_list;
  int col = 0;

  for (const auto& vars : get_variables()->get_components()) {
    Variable::ConstPtr varset = std::dynamic_pointer_cast<const Variable>(vars);
    math::SparseJacobian jac_block(get_rows(), varset->numvars_all());

    fill_jacobian_block(varset, jac_block);

    // create triplets for the derivative at the correct position in the overall
    // Jacobian
    for (int k = 0; k < jac_block.outerSize(); ++k) {
      for (math::SparseJacobian::InnerIterator it(jac_block, k); it; ++it) {
        int c = varset->map_index(it.col());
        if (c >= 0) {
          triplet_list.push_back(
              Eigen::Triplet<double>(it.row(), col + c, it.value()));
        }
      }
    }
    col += varset->numvars_free();
  }

  // transform triplet vector into sparse matrix
  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}

//==============================================================================
math::SparseJacobian Constraint::get_jacobian_finite_diff() const {
  math::SparseJacobian jacobian(get_rows(), get_variables()->get_rows());
  int col = 0;
  math::SparseJacobian jac;
  std::vector<Eigen::Triplet<double>> triplet_list;

  for (const auto& vars : get_variables()->get_components()) {
    Variable::ConstPtr varset = std::dynamic_pointer_cast<const Variable>(vars);
    jac.resize(get_rows(), varset->numvars_all());

    fill_jacobian_block_finite_diff(vars, jac);

    // create triplets for the derivative at the correct position in the overall
    // Jacobian
    for (int k = 0; k < jac.outerSize(); ++k) {
      for (math::SparseJacobian::InnerIterator it(jac, k); it; ++it) {
        int c = varset->map_index(it.col());
        if (c >= 0) {
          triplet_list.push_back(
              Eigen::Triplet<double>(it.row(), col + c, it.value()));
        }
      }
    }
    col += varset->numvars_free();
  }

  // transform triplet vector into sparse matrix
  jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return jacobian;
}
