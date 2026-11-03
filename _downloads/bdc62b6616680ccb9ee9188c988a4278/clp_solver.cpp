#ifdef CROBOT_HAVE_CLP

#  include <iostream>
#  include <stdexcept>
#  include <vector>

#  include <coin/ClpSimplex.hpp>
#  include <coin/CoinPackedMatrix.hpp>

#  include "crobot/optimization/clp_solver.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
ClpSolver::ClpSolver() : m_log_level(0) {
  m_status = OPTIMAL;
}

//==============================================================================
void ClpSolver::solve(Problem& nlp) {
  auto* lp = dynamic_cast<LinearProgram*>(&nlp);
  if (lp == nullptr) {
    throw std::invalid_argument("ClpSolver only solves LinearProgram problems");
  }
  solve(*lp);
}

//==============================================================================
void ClpSolver::solve(LinearProgram& lp) {
  const int n_vars = lp.get_num_decision_variables();
  const int n_cons = lp.get_num_constraints();

  // --- Constraint matrix A (row-major sparse -> CoinPackedMatrix) -----------
  math::SparseJacobian A = lp.get_constraint_jacobian();
  A.makeCompressed();

  CoinPackedMatrix coin_matrix(/*isColOrdered=*/false, 0, 0);
  coin_matrix.setDimensions(0, n_vars);

  for (int row = 0; row < A.outerSize(); ++row) {
    std::vector<int> indices;
    std::vector<double> values;
    for (math::SparseJacobian::InnerIterator it(A, row); it; ++it) {
      indices.push_back(static_cast<int>(it.col()));
      values.push_back(it.value());
    }
    coin_matrix.appendRow(
        static_cast<int>(indices.size()), indices.data(), values.data());
  }

  // --- Objective vector c (gradient of cost w.r.t. x, constant for LP) -----
  math::SparseJacobian cost_jac = lp.get_cost_jacobian();
  math::VectorXd c              = math::VectorXd::Zero(n_vars);
  for (int k = 0; k < cost_jac.outerSize(); ++k) {
    for (math::SparseJacobian::InnerIterator it(cost_jac, k); it; ++it) {
      c(static_cast<int>(it.col())) = it.value();
    }
  }

  // --- Variable bounds ------------------------------------------------------
  VecBound var_bounds = lp.get_variable_bounds();
  std::vector<double> col_lb(n_vars), col_ub(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    col_lb[i] = var_bounds[i].lower;
    col_ub[i] = var_bounds[i].upper;
  }

  // --- Constraint bounds (row bounds) --------------------------------------
  VecBound con_bounds = lp.get_constraint_bounds();
  std::vector<double> row_lb(n_cons), row_ub(n_cons);
  for (int i = 0; i < n_cons; ++i) {
    row_lb[i] = con_bounds[i].lower;
    row_ub[i] = con_bounds[i].upper;
  }

  // --- Load and solve -------------------------------------------------------
  ClpSimplex model;
  model.setLogLevel(m_log_level);
  model.loadProblem(
      coin_matrix,
      col_lb.data(),
      col_ub.data(),
      c.data(),
      row_lb.data(),
      row_ub.data());

  model.dual();
  m_status = model.status();

  if (m_status == OPTIMAL) {
    lp.set_variables(model.getColSolution());
  }
  else {
    std::cerr << "ClpSolver: no optimal solution found (status = " << m_status
              << ")\n";
  }
}

//==============================================================================
void ClpSolver::set_log_level(int level) {
  m_log_level = level;
}

#endif // CROBOT_HAVE_CLP
