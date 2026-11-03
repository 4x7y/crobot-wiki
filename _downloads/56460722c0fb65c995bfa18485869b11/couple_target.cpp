#include "crobot/control/couple_target.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
CoupleTargetPtr CoupleTarget::MakeShared(
    std::size_t active, std::size_t leader, double coef, TargetType type) {
  return std::make_shared<CoupleTarget>(active, leader, coef, type);
}

//==============================================================================
CoupleTarget::CoupleTarget(
    std::size_t active, std::size_t leader, double coef, TargetType type)
  : SingleTarget(type), m_active(active), m_leader(leader), m_coef(coef) {
}

//==============================================================================
double CoupleTarget::generate(
    double x,
    double x_dot,
    double x_desired,
    double x_dot_desired,
    double x_ddot_desired,
    double f_ext) {
  (void)x_dot;
  (void)x_desired;
  (void)x_dot_desired;
  (void)x_ddot_desired;
  (void)f_ext;

  return x * m_coef;
}

//==============================================================================
std::size_t CoupleTarget::get_active_id() const {
  return m_active;
}

//==============================================================================
std::size_t CoupleTarget::get_leader_id() const {
  return m_leader;
}
