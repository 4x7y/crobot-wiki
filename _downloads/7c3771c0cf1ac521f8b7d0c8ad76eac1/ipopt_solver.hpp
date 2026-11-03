#pragma once

#include "crobot/optimization/ipopt_adapter.hpp"
#include "crobot/optimization/problem.hpp"
#include "crobot/optimization/solver.hpp"

namespace Ipopt {
class IpoptApplication;
}

namespace crobot::optimization {

/**
 * @brief An interface to IPOPT, fully hiding its implementation.
 *
 * To set specific options, see:
 * https://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * @ingroup Solvers
 */
class IpoptSolver : public Solver {
public:
  using Ptr = std::shared_ptr<IpoptSolver>;

  // clang-format off
  enum Status
  {
    SUCCEEDED                          = Ipopt::Solve_Succeeded,
    SOLVED_TO_ACCEPTABLE_LEVEL         = Ipopt::Solved_To_Acceptable_Level,
    FEASIBLE_POINT_FOUND               = Ipopt::Feasible_Point_Found,
    INFEASIBLE_PROBLEM_DETECTED        = Ipopt::Infeasible_Problem_Detected,
    SEARCH_DIRECTION_BECOMES_TOO_SMALL = Ipopt::Search_Direction_Becomes_Too_Small,
    DIVERGING_ITERATES                 = Ipopt::Diverging_Iterates,
    USER_REQUESTED_STOP                = Ipopt::User_Requested_Stop,
    
    MAXIMUM_ITERATIONS_EXCEEDED        = Ipopt::Maximum_Iterations_Exceeded,
    RESTORATION_FAILED                 = Ipopt::Restoration_Failed,
    ERROR_IN_STEP_COMPUTATION          = Ipopt::Error_In_Step_Computation,
    MAXIMUM_CPUTIME_EXCEEDED           = Ipopt::Maximum_CpuTime_Exceeded,
    NOT_ENOUGH_DEGREES_OF_FREEDOM      = Ipopt::Not_Enough_Degrees_Of_Freedom,
    INVALID_PROBLEM_DEFINITION         = Ipopt::Invalid_Problem_Definition,
    INVALID_OPTION                     = Ipopt::Invalid_Option,
    INVALID_NUMBER_DETECTED            = Ipopt::Invalid_Number_Detected,
    
    UNRECOVERABLE_EXCEPTION            = Ipopt::Unrecoverable_Exception,
    NONIPOPT_EXCEPTION_THROWN          = Ipopt::NonIpopt_Exception_Thrown,
    INSUFFICIENT_MEMORY                = Ipopt::Insufficient_Memory,
    INTERNAL_ERROR                     = Ipopt::Internal_Error,
  };
  // clang-format on

  IpoptSolver();
  virtual ~IpoptSolver() = default;

  /**
   * @brief  Creates an IpoptAdapter and solves the NLP.
   * @param [in/out]  nlp  The specific problem.
   */
  void solve(Problem& nlp) override;

  /** Set options for the IPOPT solver. A complete list can be found here:
   * https://www.coin-or.org/Ipopt/documentation/node40.html
   */
  void set_option(const std::string& name, const std::string& value);
  void set_option(const std::string& name, int value);
  void set_option(const std::string& name, double value);

  /**
   * @brief  Get the total wall clock time for the optimization, including
   * function evaluations.
   */
  double get_total_wallclock_time();

private:
  std::shared_ptr<Ipopt::IpoptApplication> m_ipopt_app;
};

} // namespace crobot::optimization

