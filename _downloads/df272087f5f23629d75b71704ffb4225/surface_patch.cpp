#include "crobot/math/surface_patch.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
SurfacePatch::SurfacePatch(
    Surface::ConstPtr surface,
    const math::Vector2d& bounds_u,
    const math::Vector2d& bounds_v,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm,
    const math::Vector2d& uv_scale)
  : m_surface(std::move(surface)),
    m_bounds(std::make_pair<>(
        math::Vector2d(bounds_u(0) * uv_scale(0), bounds_v(0) * uv_scale(1)),
        math::Vector2d(bounds_u(1) * uv_scale(0), bounds_v(1) * uv_scale(1)))),
    m_bpos(bpos),
    m_rotm(rotm),
    m_uv_scale(uv_scale) {
  CROBOT_ASSERT_ERROR(m_surface != nullptr, "surface is a nullptr");
}

//==============================================================================
SurfacePatch::SurfacePatch(Surface::ConstPtr surface, YAML::Node node)
  : m_surface(std::move(surface)) {
  CROBOT_ASSERT_ERROR(m_surface != nullptr, "Surface is a nullptr");

  CROBOT_ASSERT_ERROR(node["bounds_u"].IsDefined());
  auto bounds_u = node["bounds_u"].as<std::vector<double>>();
  CROBOT_ASSERT_ERROR(bounds_u.size() == 2 && bounds_u[0] <= bounds_u[1]);

  CROBOT_ASSERT_ERROR(node["bounds_v"].IsDefined());
  auto bounds_v = node["bounds_v"].as<std::vector<double>>();
  CROBOT_ASSERT_ERROR(bounds_v.size() == 2 && bounds_v[0] <= bounds_v[1]);
  m_bounds = std::make_pair<>(
      math::Vector2d(bounds_u[0], bounds_v[0]),
      math::Vector2d(bounds_u[1], bounds_v[1]));

  m_bpos = [&]() -> math::Vector3d {
    if (node["bpos"].IsDefined()) {
      auto bpos = node["bpos"].as<std::vector<double>>();
      CROBOT_ASSERT_ERROR(
          bpos.size() == 3, "SurfacePatch: Wrong dimension of the bpos.");
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
          "SurfacePatch: Wrong dimension of the quaternion.");
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

  m_uv_scale = math::Vector2d::Ones();
}

//==============================================================================
math::Vector3d SurfacePatch::evaluate(const math::Vector2d& uv) const {
  return m_rotm
             * m_surface->evaluate(uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1))
         + m_bpos;
}

//==============================================================================
math::Matrix32 SurfacePatch::jacb(const math::Vector2d& uv) const {
  array2<math::Vector3d> deriv
      = m_surface->derivatives(1, uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1));
  math::Matrix32 jacp;
  jacp.col(0) = deriv(1, 0) / m_uv_scale(0);
  jacp.col(1) = deriv(0, 1) / m_uv_scale(1);
  return m_rotm * jacp;
}

//==============================================================================
math::Matrix6X SurfacePatch::hess(const math::Vector2d& uv) const {
  array2<math::Vector3d> deriv
      = m_surface->derivatives(2, uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1));
  math::Matrix6X hess = math::Matrix6X::Zero(6, 2);
  hess.block<3, 1>(0, 0)
      = m_rotm * deriv(2, 0) / (m_uv_scale(0) * m_uv_scale(0));
  hess.block<3, 1>(0, 1)
      = m_rotm * deriv(1, 1) / (m_uv_scale(0) * m_uv_scale(1));
  hess.block<3, 1>(3, 0)
      = m_rotm * deriv(1, 1) / (m_uv_scale(0) * m_uv_scale(1));
  hess.block<3, 1>(3, 1)
      = m_rotm * deriv(0, 2) / (m_uv_scale(1) * m_uv_scale(1));
  return hess;
}

//==============================================================================
array2<math::Vector3d> SurfacePatch::derivatives(
    int degree, const math::Vector2d& uv) const {
  array2<math::Vector3d> deriv = m_surface->derivatives(
      degree, uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1));

  // go through all the derivatives and scale them
  for (int i = 0; i < deriv.rows(); ++i) {
    for (int j = 0; j < deriv.cols(); ++j) {
      deriv(i, j) /= (std::pow(m_uv_scale(0), i) * std::pow(m_uv_scale(1), j));
    }
  }

  return deriv;
}

//==============================================================================
math::Matrix32 SurfacePatch::jacr(const math::Vector2d& uv) const {
  double u = uv(0);
  double v = uv(1);

  math::Matrix3d rot_u
      = get_frame(math::Vector2d(u - 1e-6, v)).block<3, 3>(0, 0).transpose()
        * get_frame(math::Vector2d(u + 1e-6, v)).block<3, 3>(0, 0);
  math::Matrix3d rot_v
      = get_frame(math::Vector2d(u, v - 1e-6)).block<3, 3>(0, 0).transpose()
        * get_frame(math::Vector2d(u, v + 1e-6)).block<3, 3>(0, 0);
  math::AngleAxisd aa_u(rot_u);
  math::AngleAxisd aa_v(rot_v);
  math::Matrix32 jacr;
  jacr.col(0) = aa_u.axis() * aa_u.angle() / (2e-6);
  jacr.col(1) = aa_v.axis() * aa_v.angle() / (2e-6);
  return m_rotm * jacr;
}

//==============================================================================
math::Vector3d SurfacePatch::normal(const math::Vector2d& uv) const {
  return m_rotm
         * m_surface->normal(uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1));
}

//==============================================================================
math::Matrix32 SurfacePatch::normal_jacb(const math::Vector2d& uv) const {
  math::Matrix32 jacp
      = m_surface->normal_jacb(uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1));
  jacp.col(0) /= m_uv_scale(0);
  jacp.col(1) /= m_uv_scale(1);
  return m_rotm * jacp;
}

//==============================================================================
math::Matrix6X SurfacePatch::normal_hess(const math::Vector2d& uv) const {
  math::Matrix6X hess
      = m_surface->normal_hess(uv(0) / m_uv_scale(0), uv(1) / m_uv_scale(1));

  hess.block<3, 1>(0, 0)
      = m_rotm * hess.block<3, 1>(0, 0) / (m_uv_scale(0) * m_uv_scale(0));
  hess.block<3, 1>(0, 1)
      = m_rotm * hess.block<3, 1>(0, 1) / (m_uv_scale(0) * m_uv_scale(1));
  hess.block<3, 1>(3, 0)
      = m_rotm * hess.block<3, 1>(3, 0) / (m_uv_scale(0) * m_uv_scale(1));
  hess.block<3, 1>(3, 1)
      = m_rotm * hess.block<3, 1>(3, 1) / (m_uv_scale(1) * m_uv_scale(1));

  return hess;
}

//==============================================================================
const math::Vector2d& SurfacePatch::get_lb() const {
  return m_bounds.first;
}

//==============================================================================
const math::Vector2d& SurfacePatch::get_ub() const {
  return m_bounds.second;
}

//==============================================================================
math::Vector2d SurfacePatch::get_u_bounds() const {
  math::Vector2d ret = math::Vector2d(m_bounds.first(0), m_bounds.second(0));
  return ret;
}

//==============================================================================
math::Vector2d SurfacePatch::get_v_bounds() const {
  math::Vector2d ret = math::Vector2d(m_bounds.first(1), m_bounds.second(1));
  return ret;
}

//==============================================================================
math::Matrix4d SurfacePatch::get_frame(const math::Vector2d& uv) const {

  math::Vector3d bpos = evaluate(uv);
  math::Vector3d nvec = normal(uv);

  math::Matrix32 tbvec = jacb(uv);
  math::Vector3d tvec  = tbvec.col(0).normalized();
  math::Vector3d bvec  = nvec.cross(tvec);
  math::Matrix3d rotm;
  rotm.col(0) = tvec;
  rotm.col(1) = bvec;
  rotm.col(2) = nvec;

  math::Matrix4d tform    = rotm.rotm2tform();
  tform.block<3, 1>(0, 3) = bpos;
  return tform;
}

//==============================================================================
void SurfacePatch::update(
    Surface::ConstPtr surface,
    const math::Vector2d& bounds_u,
    const math::Vector2d& bounds_v,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm,
    const math::Vector2d& uv_scale) {
  m_surface = surface;
  m_bounds  = std::make_pair<>(
      math::Vector2d(bounds_u(0) * uv_scale(0), bounds_v(0) * uv_scale(1)),
      math::Vector2d(bounds_u(1) * uv_scale(0), bounds_v(1) * uv_scale(1)));
  m_bpos     = bpos;
  m_rotm     = rotm;
  m_uv_scale = uv_scale;
}

//==============================================================================
bool SurfacePatch::is_boundary(const math::Vector2d& uv) const {
  if (uv(0) <= m_bounds.first(0) || uv(1) <= m_bounds.first(1)
      || uv(0) >= m_bounds.second(0) || uv(1) >= m_bounds.second(1)) {
    // std::cout << "patch out of boundary: " << uv.transpose() << std::endl;
    return true;
  }
  else {
    return false;
  }
}

//==============================================================================
void SurfacePatch::rotate(const math::Matrix3d& rotm) {
  m_rotm = rotm * m_rotm;
}

//==============================================================================
void SurfacePatch::translate(const math::Vector3d& bpos) {
  m_bpos += bpos;
}
//==============================================================================
void SurfacePatch::set_bpos(const math::Vector3d& bpos) {
  m_bpos = bpos;
}

//==============================================================================
void SurfacePatch::set_rotm(const math::Matrix3d& rotm) {
  m_rotm = rotm;
}

//==============================================================================
math::Matrix4d SurfacePatch::get_tform() const {
  math::Matrix4d tform    = m_rotm.rotm2tform();
  tform.block<3, 1>(0, 3) = m_bpos;
  return tform;
}
