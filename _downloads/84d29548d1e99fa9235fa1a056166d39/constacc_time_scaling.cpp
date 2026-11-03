#include "crobot/planning/constacc_time_scaling.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
ConstaccTimeScaling::ConstaccTimeScaling(double acc)
  : TimeScaling(2 * std::sqrt(1.0 / acc)), m_acc(acc) {
}

//==============================================================================
double ConstaccTimeScaling::get_scaling_factor(double time) const {
  if (time < 0.0) {
    return 0.0;
  }
  else if (time < m_duration / 2.0) {
    return 0.5 * m_acc * time * time;
  }
  else if (time < m_duration) {
    return 1 - 0.5 * m_acc * (m_duration - time) * (m_duration - time);
  }
  else {
    return 1.0;
  }
}

//==============================================================================
double ConstaccTimeScaling::get_scaling_factor_deriv(double time) const {
  if (time < 0.0) {
    return 0.0;
  }
  else if (time < m_duration / 2.0) {
    return m_acc * time;
  }
  else if (time < m_duration) {
    return m_acc * (m_duration - time);
  }
  else {
    return 1.0;
  }
}

//==============================================================================
double ConstaccTimeScaling::get_scaling_factor_deriv2(double time) const {
  if (time < 0.0) {
    return 0.0;
  }
  else if (time < m_duration / 2.0) {
    return m_acc;
  }
  else if (time < m_duration) {
    return -m_acc;
  }
  else {
    return 1.0;
  }
}
