#include "crobot/planning/linear_time_scaling.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
LinearTimeScaling::LinearTimeScaling(double time_start, double time_end)
  : TimeScaling(time_end - time_start) {
  CROBOT_ASSERT_ERROR(
      time_start < time_end, "time_start must be less than time_end.");

  m_scale         = 1. / (time_end - time_start);
  m_time_interval = std::make_pair(time_start, time_end);
}

//==============================================================================
LinearTimeScaling::LinearTimeScaling(double duration)
  : LinearTimeScaling(0.0, duration) {
}

//==============================================================================
double LinearTimeScaling::get_scaling_factor(double time) const {
  if (time < m_time_interval.first) {
    return 0.0;
  }
  else if (time > m_time_interval.second) {
    return 1.0;
  }
  else {
    return m_scale * (time - m_time_interval.first);
  }
}

//==============================================================================
double LinearTimeScaling::get_scaling_factor_deriv(double time) const {
  if (time < m_time_interval.first || time > m_time_interval.second) {
    return 0.0;
  }
  else {
    return m_scale;
  }
}

//==============================================================================
double LinearTimeScaling::get_scaling_factor_deriv2(double /* time */) const {
  return 0.0;
}
