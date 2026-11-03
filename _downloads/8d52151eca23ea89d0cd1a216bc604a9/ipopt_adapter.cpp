#include "crobot/optimization/ipopt_adapter.hpp"

#include "crobot/math/matrix.hpp"

using namespace Ipopt;
using namespace crobot;

//==============================================================================
IpoptAdapter::IpoptAdapter(Problem& nlp, bool finite_diff) {
  m_nlp            = &nlp;
  m_is_finite_diff = finite_diff;
}

//==============================================================================
bool IpoptAdapter::get_nlp_info(
    Index& n,
    Index& m,
    Index& nnz_jac_g,
    Index& nnz_h_lag,
    IndexStyleEnum& index_style) {
  n = m_nlp->get_num_variables();
  m = m_nlp->get_num_constraints();

  if (m_is_finite_diff)
    nnz_jac_g = m * n;
  else
    nnz_jac_g = m_nlp->get_constraint_jacobian().nonZeros();

  nnz_h_lag = n * n;

  // start index at 0 for row/col entries
  index_style = C_STYLE;

  return true;
}

//==============================================================================
bool IpoptAdapter::get_bounds_info(
    Index /* n */,
    double* x_lower,
    double* x_upper,
    Index /* m */,
    double* g_l,
    double* g_u) {
  auto bounds_x = m_nlp->get_variable_bounds();
  for (std::size_t c = 0; c < bounds_x.size(); ++c) {
    x_lower[c] = bounds_x.at(c).lower;
    x_upper[c] = bounds_x.at(c).upper;
  }

  // specific bounds depending on equality and inequality constraints
  auto bounds_g = m_nlp->get_constraint_bounds();
  for (std::size_t c = 0; c < bounds_g.size(); ++c) {
    g_l[c] = bounds_g.at(c).lower;
    g_u[c] = bounds_g.at(c).upper;
  }

  return true;
}

//==============================================================================
bool IpoptAdapter::get_starting_point(
    Index /* n */,
    bool init_x,
    double* x,
    bool init_z,
    double* /* z_L */,
    double* /* z_U */,
    Index /* m */,
    bool init_lambda,
    double* /* lambda */) {
  // Here, we assume we only have starting values for x
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  (void)init_x;
  (void)init_z;
  (void)init_lambda;

  math::VectorXd x_all                     = m_nlp->get_variable_values();
  math::MapToVectorXd(&x[0], x_all.rows()) = x_all;

  return true;
}

//==============================================================================
bool IpoptAdapter::eval_f(
    Index /* n */, const double* x, bool /* new_x */, double& obj_value) {
  obj_value = m_nlp->evaluate_cost_function(x);
  return true;
}

//==============================================================================
bool IpoptAdapter::eval_grad_f(
    Index n, const double* x, bool /* new_x */, double* grad_f) {
  Eigen::VectorXd grad
      = m_nlp->evaluate_cost_function_gradient(x, m_is_finite_diff);
  Eigen::Map<Eigen::MatrixXd>(grad_f, n, 1) = grad;
  return true;
}

//==============================================================================
bool IpoptAdapter::eval_g(
    Index /* n */, const double* x, bool /* new_x */, Index m, double* g) {
  math::VectorXd g_eig      = m_nlp->evaluate_constraints(x);
  math::MapToVectorXd(g, m) = g_eig;
  return true;
}

//==============================================================================
bool IpoptAdapter::eval_jac_g(
    Index n,
    const double* x,
    bool /* new_x */,
    Index m,
    Index nele_jac,
    Index* iRow,
    Index* jCol,
    double* values) {
  (void)nele_jac;
  // defines the positions of the nonzero elements of the jacobian
  if (values == NULL) {
    // If "jacobian_approximation" option is set as "finite-difference-values",
    // the Jacobian is dense!
    Index nele = 0;
    if (m_is_finite_diff) { // dense jacobian
      for (Index row = 0; row < m; row++) {
        for (Index col = 0; col < n; col++) {
          iRow[nele] = row;
          jCol[nele] = col;
          nele++;
        }
      }
    }
    else { // sparse jacobian
      auto jac = m_nlp->get_constraint_jacobian();
      for (int k = 0; k < jac.outerSize(); ++k) {
        for (math::SparseJacobian::InnerIterator it(jac, k); it; ++it) {
          iRow[nele] = it.row();
          jCol[nele] = it.col();
          nele++;
        }
      }
    }
    // initial sparsity structure is never allowed to change
    assert(nele == nele_jac);
  }
  else {
    // only gets used if "jacobian_approximation finite-difference-values" is
    // not set
    m_nlp->evaluate_jacobian_nonzeros(x, values);
  }

  return true;
}

//==============================================================================
bool IpoptAdapter::intermediate_callback(
    AlgorithmMode /* mode */,
    Index /* iter */,
    double /* obj_value */,
    double /* inf_pr */,
    double /* inf_du */,
    double /* mu */,
    double /* d_norm */,
    double /* regularization_size */,
    double /* alpha_du */,
    double /* alpha_pr */,
    Index /* ls_trials */,
    const IpoptData* /* ip_data */,
    IpoptCalculatedQuantities* /* ip_cq */) {
  m_nlp->save_current();
  return true;
}

//==============================================================================
void IpoptAdapter::finalize_solution(
    SolverReturn /* status */,
    Index /* n */,
    const double* x,
    const double* /* z_L */,
    const double* /* z_U */,
    Index /* m */,
    const double* /* g */,
    const double* /* lambda */,
    double /* obj_value */,
    const IpoptData* /* ip_data */,
    IpoptCalculatedQuantities* /* ip_cq */) {
  m_nlp->set_variables(x);
  m_nlp->save_current();
}
