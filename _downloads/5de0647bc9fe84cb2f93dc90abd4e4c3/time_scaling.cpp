#include "crobot/planning/time_scaling.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
TimeScaling::TimeScaling(double duration) : m_duration(duration) {
}

//==============================================================================
double TimeScaling::get_duration() const {
  return m_duration;
}

//==============================================================================
std::vector<std::vector<double>>
crobot::planning::construct_linear_time_scaling(
    const std::pair<double, double>& time_interval,
    const std::vector<double>& time) {

  // Ensure that time is a row
  std::size_t m = time.size();
  std::vector<double> s, sd, sdd;
  s.reserve(m);
  sd.reserve(m);
  sdd.reserve(m);

  // Use interval to compute position values
  double linearScaling = 1 / (time_interval.second - time_interval.first);
  for (auto t : time) {
    // Saturate position and velocity outside the time interval
    if (t < time_interval.first) {
      s.push_back(0.0);
      sd.push_back(0.0);
    }
    else if (t > time_interval.second) {
      s.push_back(1.0);
      sd.push_back(0.0);
    }
    else {
      s.push_back(linearScaling * (t - time_interval.first));
      sd.push_back(linearScaling);
    }
    sdd.push_back(0.0);
  }

  // Build output
  return {s, sd, sdd};
}
