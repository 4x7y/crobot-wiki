#include "crobot/optimization/ipopt_solver.hpp"

#include "crobot/optimization/ipopt_adapter.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
IpoptSolver::IpoptSolver() {
  m_ipopt_app = std::make_shared<Ipopt::IpoptApplication>();
  m_status    = Ipopt::Solve_Succeeded;

  /* Which linear solver to use. Mumps is default because it comes with the
   * precompiled ubuntu binaries. However, the coin-hsl solvers can be
   * significantly faster and are free for academic purposes. They can be
   * downloaded here: http://www.hsl.rl.ac.uk/ipopt/ and must be compiled
   * into your IPOPT libraries. Then you can use the additional strings:
   * "ma27, ma57, ma77, ma86, ma97" here.
   */
  set_option("linear_solver", "mumps");

  /* whether to use the analytical derivatives "exact" coded in ifopt, or let
   * IPOPT approximate these through "finite difference-values". This is usually
   * significantly slower.
   */
  set_option("jacobian_approximation", "exact");
  set_option("hessian_approximation", "limited-memory");
  set_option("max_cpu_time", 400.0);
  set_option("tol", 0.001);
  set_option("print_timing_statistics", "no");
  set_option("print_user_options", "no");
  set_option("print_level", 4);

  // set_option("max_iter", 1);
  // set_option("derivative_test", "first-order");
  // set_option("derivative_test_tol", 1e-3);
}

//==============================================================================
void IpoptSolver::solve(Problem& nlp) {
  using namespace Ipopt;

  m_status = m_ipopt_app->Initialize();
  if (m_status != Solve_Succeeded) {
    std::cout << std::endl
              << std::endl
              << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  // check the jacobian_approximation method
  std::string jac_type = "";
  m_ipopt_app->Options()->GetStringValue(
      "jacobian_approximation", jac_type, "");
  bool finite_diff = jac_type == "finite-difference-values";

  // convert the NLP problem to Ipopt
  SmartPtr<TNLP> nlp_ptr = new IpoptAdapter(nlp, finite_diff);
  m_status               = m_ipopt_app->OptimizeTNLP(nlp_ptr);

  if (m_status != Solve_Succeeded) {
    std::string msg = "ERROR: Ipopt failed to find a solution. Return Code: "
                      + std::to_string(m_status) + "\n";
    std::cerr << msg;
  }
}

//==============================================================================
void IpoptSolver::set_option(
    const std::string& name, const std::string& value) {
  m_ipopt_app->Options()->SetStringValue(name, value);
}

//==============================================================================
void IpoptSolver::set_option(const std::string& name, int value) {
  m_ipopt_app->Options()->SetIntegerValue(name, value);
}

//==============================================================================
void IpoptSolver::set_option(const std::string& name, double value) {
  m_ipopt_app->Options()->SetNumericValue(name, value);
}

//==============================================================================
double IpoptSolver::get_total_wallclock_time() {
  return m_ipopt_app->Statistics()->TotalWallclockTime();
}
