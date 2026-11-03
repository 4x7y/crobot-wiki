#include "crobot/ik/cost.hpp"

#include <iomanip>
#include <iostream>

using namespace crobot;
using namespace ik;

//==============================================================================
Cost::Cost(const std::string& name, const std::vector<std::string>& variables)
  : Constraint(1, name, variables) {
}

//==============================================================================
math::VectorXd Cost::get_values(const math::VectorXd& x) const {
  math::VectorXd cost(1);
  cost(0) = get_cost(x);
  return cost;
}

//==============================================================================
optimization::VecBound Cost::get_bounds() const {
  return optimization::VecBound(get_rows(), optimization::NoBound);
}

//==============================================================================
void Cost::show_info(double /* tol */, int& index) const {
  // only one scalar cost value
  double cost = get_values()(0);

  std::cout.precision(2);
  std::cout << std::fixed << std::left << std::setw(30) << get_name()
            << std::right << std::setw(4) << get_rows() << std::setw(9) << index
            << std::setfill('.') << std::setw(7) << index + get_rows() - 1
            << std::setfill(' ') << std::setw(12) << cost << std::endl;
}
