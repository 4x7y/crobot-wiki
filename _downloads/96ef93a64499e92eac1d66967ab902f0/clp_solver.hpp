#pragma once

#ifdef CROBOT_HAVE_CLP

#  include <memory>

#  include "crobot/optimization/linear_program.hpp"
#  include "crobot/optimization/solver.hpp"

namespace crobot::optimization {

/**
 * @brief Solver for linear programs using the COIN-OR CLP dual simplex.
 *
 * Only accepts LinearProgram problems. Calling solve(Problem&) with a
 * non-LinearProgram argument throws std::invalid_argument.
 *
 * @ingroup Solvers
 */
class ClpSolver : public Solver {
public:
  using Ptr = std::shared_ptr<ClpSolver>;

  enum Status
  {
    OPTIMAL              = 0,
    PRIMAL_INFEASIBLE    = 1,
    DUAL_INFEASIBLE      = 2,
    STOPPED_ON_ITERATION = 3,
    STOPPED_ON_ERROR     = 4,
  };

  ClpSolver();
  virtual ~ClpSolver() = default;

  /**
   * @brief Solves the linear program using CLP's dual simplex method.
   * @param [in/out] lp  The linear programming problem to solve.
   */
  void solve(LinearProgram& lp);

  /**
   * @brief Satisfies the Solver interface. Delegates to solve(LinearProgram&)
   * via dynamic_cast; throws std::invalid_argument if @p nlp is not an LP.
   */
  void solve(Problem& nlp) override;

  /**
   * @brief Set the CLP log level (0 = silent, 4 = verbose).
   */
  void set_log_level(int level);

private:
  int m_log_level;
};

} // namespace crobot::optimization

#endif // CROBOT_HAVE_CLP
