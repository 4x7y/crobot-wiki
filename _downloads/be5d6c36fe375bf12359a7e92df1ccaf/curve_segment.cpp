#include "crobot/common/assert.hpp"
#include "crobot/math/curve_segment.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
CurveSegment::CurveSegment(
    Curve::ConstPtr curve,
    const math::Vector2d& bounds_u,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm)
  : m_curve(curve),
    m_bounds(std::make_pair(bounds_u[0], bounds_u[1])),
    m_bpos(bpos),
    m_rotm(rotm) {
  CROBOT_ASSERT_ERROR(m_curve != nullptr, "curve is a nullptr");
}

//==============================================================================
CurveSegment::CurveSegment(Curve::ConstPtr curve, YAML::Node node)
  : m_curve(curve) {
  CROBOT_ASSERT_ERROR(m_curve != nullptr, "curve is a nullptr");

  CROBOT_ASSERT_ERROR(node["bounds"].IsDefined());
  auto bounds = node["bounds"].as<std::vector<double>>();
  CROBOT_ASSERT_ERROR(bounds.size() == 2 && bounds[0] <= bounds[1]);
  m_bounds = std::make_pair(bounds[0], bounds[1]);

  m_bpos = [&]() -> math::Vector3d {
    if (node["bpos"].IsDefined()) {
      auto bpos = node["bpos"].as<std::vector<double>>();
      CROBOT_ASSERT_ERROR(
          bpos.size() == 3, "CurveSegment: Wrong dimension of the bpos.");
      return Eigen::Map<Eigen::Vector3d>(bpos.data(), bpos.size());
    }
    else {
      return math::Vector3d::Zero();
    }
  }();

  m_rotm = [&]() -> math::Matrix3d {
    if (node["quat"].IsDefined()) {
      auto quat_coeffs = node["quat"].as<std::vector<double>>();
      CROBOT_ASSERT_ERROR(
          quat_coeffs.size() == 4,
          "CurveSegment: Wrong dimension of the quaternion.");
      math::Quaternion quat(
          quat_coeffs[0],  // w
          quat_coeffs[1],  // x
          quat_coeffs[2],  // y
          quat_coeffs[3]); // z
      return quat.as_rotm();
    }
    else {
      return math::Matrix3d::Identity();
    }
  }();
}

//==============================================================================
math::Vector3d CurveSegment::evaluate(double u) const {
  return m_rotm * m_curve->evaluate(u) + m_bpos;
}

//==============================================================================
math::Vector3d CurveSegment::tangent(double u) const {
  return m_curve->tangent(u);
}

//==============================================================================
math::Vector3d CurveSegment::jacb(double u) const {
  std::vector<math::Vector3d> deriv = m_curve->derivatives(1, u);
  return m_rotm * deriv[0];
}

//==============================================================================
math::Vector3d CurveSegment::tangent_jacb(double u) const {
  return m_rotm * m_curve->tangent_jacb(u);
}

//==============================================================================
double CurveSegment::get_lb() const {
  return m_bounds.first;
}

//==============================================================================
double CurveSegment::get_ub() const {
  return m_bounds.second;
}
