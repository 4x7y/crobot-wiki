#include <iomanip>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/component.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
Component::Component(int num_rows, const std::string& name) {
  m_num_rows = num_rows;
  m_name     = name;
}

//==============================================================================
int Component::get_rows() const {
  return m_num_rows;
}

//==============================================================================
void Component::set_rows(int num_rows) {
  m_num_rows = num_rows;
}

//==============================================================================
std::string Component::get_name() const {
  return m_name;
}

//==============================================================================
void Component::show_info(double tol, int& index) const {
  // calculate squared bound violation
  VecBound bounds          = get_bounds();
  crobot::math::VectorXd x = get_values();

  std::vector<std::pair<int, double>> viol_idx;
  for (std::size_t i = 0; i < bounds.size(); ++i) {
    double lower = bounds.at(i).lower;
    double upper = bounds.at(i).upper;
    double val   = x(static_cast<int>(i));
    if (val < lower - tol)
      viol_idx.push_back(
          std::make_pair<>(i, lower - val)); // violates lower bounds
    if (upper + tol < val)
      viol_idx.push_back(
          std::make_pair<>(i, val - upper)); // violates higher bounds
  }

  std::string black = "\033[0m";
  std::string red   = "\033[31m";
  std::string color = viol_idx.empty() ? black : red;

  std::cout.precision(2);
  std::cout << std::fixed << std::left << std::setw(30) << m_name << std::right
            << std::setw(4) << m_num_rows << std::setw(9) << index
            << std::setfill('.') << std::setw(7) << index + m_num_rows - 1
            << std::setfill(' ') << color << std::setw(12) << viol_idx.size()
            << black;

  if (!viol_idx.empty()) {
    double val = 0.;
    for (size_t i = 0; i < viol_idx.size(); i++) {
      val += viol_idx[i].second * viol_idx[i].second;
    }
    val = sqrt(val);
    std::cout << std::setfill(' ') << std::setw(10) << std::setprecision(6)
              << std::fixed << val;
  }

  std::cout << std::endl;

  index += m_num_rows;
}
