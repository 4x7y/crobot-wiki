#pragma once
#include <memory>

#include "crobot/optimization/qp_problem.hpp"
#include "crobot/optimization/solver.hpp"

namespace crobot::optimization {

class QPSolver : public Solver {
public:
  using Ptr = std::shared_ptr<QPSolver>;

  virtual ~QPSolver() = default;

  /**
   * @brief Uses a specific solver (e.g. OSQP) to solve the QP.
   * @param [in/out]  nlp  The quadratic programming problem.
   */
  virtual void solve(QPProblem& qp) = 0;

  /**
   * @brief Get the return status for the optimization.
   *
   * e.g. https://osqp.org/docs/interfaces/solver_settings.html#return-codes
   */
  int get_return_status() const {
    return m_status;
  };
};

} // namespace crobot::optimization
