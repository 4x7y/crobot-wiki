#include "crobot/kdl/coupling.hpp"

#include "crobot/math/polynomial.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
Coupling::Coupling(
    const std::string& name,
    ConstJointPtr joint1,
    ConstJointPtr joint2,
    const math::VectorXd& poly)
  : m_name(name),
    m_joint1(std::move(joint1)),
    m_joint2(std::move(joint2)),
    m_poly(poly) {
  m_polyder = math::polyder(m_poly);
}

//==============================================================================
std::pair<double, double> Coupling::get_qvel_coeffs(double qpos2) const {
  return std::make_pair<>(-1, math::polyval(m_polyder, qpos2));
}
