#include "crobot/planning/trapvel_time_scaling.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
TrapvelTimeScaling::TrapvelTimeScaling(double duration)
  : TimeScaling(duration) {
}

//==============================================================================
double TrapvelTimeScaling::get_scaling_factor(double time) const {
  if (time < 0.0) {
    return 0.0;
  }
  else if (time < m_duration / 4.0) {
    return (8.0 * time * time) / (3.0 * m_duration * m_duration);
  }
  else if (time < m_duration / 4.0 * 3.0) {
    return (8.0 * time - m_duration) / (6.0 * m_duration);
  }
  else if (time < m_duration) {
    return (-5.0 * m_duration * m_duration + 16.0 * m_duration * time
            - 8.0 * time * time)
           / (3.0 * m_duration * m_duration);
  }
  else {
    return 1.0;
  }
}

//==============================================================================
double TrapvelTimeScaling::get_scaling_factor_deriv(double time) const {
  if (time < 0.0) {
    return 0.0;
  }
  else if (time < m_duration / 4.0) {
    return 16.0 * time / (3.0 * m_duration * m_duration);
  }
  else if (time < m_duration / 4.0 * 3.0) {
    return 4.0 / (3.0 * m_duration);
  }
  else if (time < m_duration) {
    return (16.0 * m_duration - 16.0 * time) / (3.0 * m_duration * m_duration);
  }
  else {
    return 0.0;
  }
}

//==============================================================================
double TrapvelTimeScaling::get_scaling_factor_deriv2(double time) const {
  if (time < 0.0) {
    return 0.0;
  }
  else if (time < m_duration / 4.0) {
    return 16.0 / (3.0 * m_duration * m_duration);
  }
  else if (time < m_duration / 4.0 * 3.0) {
    return 0.0;
  }
  else if (time < m_duration) {
    return -16.0 / (3.0 * m_duration * m_duration);
  }
  else {
    return 0.0;
  }
}
