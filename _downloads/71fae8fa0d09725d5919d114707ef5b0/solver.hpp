#pragma once

#include <memory>

#include "crobot/optimization/problem.hpp"

namespace crobot::optimization {

/**
 * @brief Solver interface implemented by IPOPT.
 */
class Solver {
public:
  using Ptr = std::shared_ptr<Solver>;

  virtual ~Solver() = default;

  /**
   * @brief Uses a specific solver (IPOPT) to solve the NLP.
   * @param [in/out]  nlp  The nonlinear programming problem.
   */
  virtual void solve(Problem& nlp) = 0;

  /**
   * @brief Get the return status for the optimization.
   *
   * e.g. https://coin-or.github.io/Ipopt/OUTPUT.html
   */
  int get_return_status() const {
    return m_status;
  };

protected:
  int m_status;
};

} // namespace crobot::optimization

