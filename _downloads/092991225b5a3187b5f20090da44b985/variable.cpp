#include "crobot/ik/variable.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
Variable::Variable(int n_var, const std::string& name)
  : optimization::VariableSet(n_var, name),
    m_vars(math::VectorXd::Zero(n_var)),
    m_indices(math::VectorXi::LinSpaced(n_var, 0, n_var - 1)) {
}

//==============================================================================
int Variable::numvars_all() const {
  return m_vars.size();
}

//==============================================================================
int Variable::numvars_fixed() const {
  return m_vars.size() - numvars_free();
}

//==============================================================================
int Variable::numvars_free() const {
  return get_rows();
}

//==============================================================================
void Variable::fix_vars(
    const std::pair<int, int>& range, const math::VectorXd& value) {
  CROBOT_ASSERT_ERROR(
      range.first >= 0 && //
      range.second > 0 && //
      range.first + range.second <= m_vars.size());
  CROBOT_ASSERT_ERROR(value.size() == range.second);

  // update m_indices
  int new_fixed = 0;
  for (int i = range.first; i < range.first + range.second; i++) {
    if (m_indices[i] >= 0) {
      m_indices[i] = -1;
      new_fixed++;
    }
  }
  for (int i = range.first + range.second; i < m_indices.size(); i++) {
    if (m_indices[i] >= 0) {
      m_indices[i] = m_indices[i] - new_fixed;
    }
  }

  // update m_vars
  m_vars.segment(range.first, range.second) = value;

  // update rows
  set_rows(get_rows() - new_fixed);
}

//==============================================================================
std::vector<int> Variable::get_freevar_indices() const {
  std::vector<int> indices_freevar;
  for (int i = 0; i < m_indices.size(); i++) {
    if (m_indices[i] >= 0) {
      indices_freevar.push_back(i);
    }
  }
  return indices_freevar;
}

//==============================================================================
int Variable::map_index(int index) const {
  return m_indices[index];
}

//==============================================================================
void Variable::set_variables(const math::VectorXd& vars_free) {
  CROBOT_ASSERT_ERROR(vars_free.size() == numvars_free());
  for (int i = 0, j = 0; i < m_vars.size(); i++) {
    if (m_indices[i] >= 0) {
      m_vars[i] = vars_free[j];
      j++;
    }
  }
  set_values_all(m_vars);
}

//==============================================================================
math::VectorXd Variable::get_values() const {
  math::VectorXd vars_all = get_values_all();
  math::VectorXd vars_free(numvars_free());
  for (int i = 0; i < vars_all.size(); i++) {
    if (m_indices[i] >= 0) {
      vars_free[m_indices[i]] = vars_all[i];
    }
  }
  return vars_free;
}

//==============================================================================
optimization::VecBound Variable::get_bounds() const {
  optimization::VecBound bounds_all = get_bounds_all();
  optimization::VecBound bounds_free(numvars_free());
  for (int i = 0; i < numvars_all(); i++) {
    if (m_indices[i] >= 0) {
      bounds_free[m_indices[i]] = bounds_all[i];
    }
  }
  return bounds_free;
}
