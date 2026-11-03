#include "crobot/math/surface_patch_switch.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/math/bspline_surface.hpp"
#include "crobot/math/ellipsoid.hpp"
#include "crobot/math/plane.hpp"
#include "crobot/math/sphere.hpp"

using namespace crobot;
using namespace math;

//==============================================================================

// check bounds uv!!!!!!! PROBLEM!!!!!
SurfacePatchSwitch::SurfacePatchSwitch(
    const std::vector<Surface::ConstPtr>& surfaces,
    const std::vector<math::Vector2d>& bounds_u,
    const std::vector<math::Vector2d>& bounds_v,
    const std::vector<math::Vector3d>& bpos,
    const std::vector<math::Matrix3d>& rotm,
    const std::vector<math::Vector2d>& uv_scale,
    std::size_t active_patch_id)
  : SurfacePatch(
      surfaces[active_patch_id],
      bounds_u[active_patch_id],
      bounds_v[active_patch_id],
      bpos[active_patch_id],
      rotm[active_patch_id],
      uv_scale[active_patch_id]),
    m_surface_all(surfaces),
    m_bpos_all(bpos),
    m_rotm_all(rotm),
    m_idx_active(active_patch_id) {

  for (size_t i = 0; i < surfaces.size(); ++i) {
    CROBOT_ASSERT_ERROR(surfaces[i] != nullptr, "surface is a nullptr");
    m_bounds_all.push_back(std::make_pair<>(bounds_u[i], bounds_v[i]));
  }

  if (uv_scale.size() != surfaces.size()) {
    CROBOT_ASSERT_WARNING(
        true,
        "The size of uv_scale is not equal to the number of surfaces. "
        "Setting the same uv_scale = 1.0 for all the surfaces.");
    m_uv_scale_all
        = std::vector<math::Vector2d>(surfaces.size(), math::Vector2d::Ones());
  }
  else {
    m_uv_scale_all = uv_scale;
  }
}

//==============================================================================
SurfacePatchSwitch::SurfacePatchSwitch(
    const std::vector<Surface::ConstPtr>& surfaces,
    const std::vector<std::pair<math::Vector2d, math::Vector2d>>& bounds,
    const std::vector<math::Vector3d>& bpos,
    const std::vector<math::Matrix3d>& rotm,
    const std::vector<math::Vector2d>& uv_scale,
    std::size_t active_patch_id)
  : SurfacePatch(
      surfaces[active_patch_id],
      bounds[active_patch_id].first,
      bounds[active_patch_id].second,
      bpos[active_patch_id],
      rotm[active_patch_id],
      uv_scale[active_patch_id]),
    m_surface_all(surfaces),
    m_bounds_all(bounds),
    m_bpos_all(bpos),
    m_rotm_all(rotm),
    m_uv_scale_all(uv_scale),
    m_idx_active(active_patch_id) {
}

//==============================================================================
std::shared_ptr<SurfacePatchSwitch> SurfacePatchSwitch::from_yaml_node(
    const YAML::Node& node) {
  CROBOT_ASSERT_ERROR(
      node["type"].as<std::string>() == "SurfacePatchSwitch",
      "type is not SurfacePatchSwitch");

  std::size_t idx_active = node["active_patch_id"].as<std::size_t>();

  std::vector<Surface::ConstPtr> surfaces;
  std::vector<math::Vector2d> bounds_u;
  std::vector<math::Vector2d> bounds_v;
  std::vector<math::Vector3d> bpos;
  std::vector<math::Matrix3d> rotm;
  std::vector<math::Vector2d> uv_scale;

  for (std::size_t i = 0; i < node["patches"].size(); ++i) {
    YAML::Node patch_node = node["patches"][i];

    if (patch_node["surface"]["type"].as<std::string>() == "BSplineSurface") {
      surfaces.push_back(BSplineSurface::from_yaml_node(patch_node["surface"]));
    }
    else if (patch_node["surface"]["type"].as<std::string>() == "Sphere") {
      surfaces.push_back(Sphere::from_yaml_node(patch_node["surface"]));
    }
    else if (patch_node["surface"]["type"].as<std::string>() == "Ellipsoid") {
      surfaces.push_back(Ellipsoid::from_yaml_node(patch_node["surface"]));
    }
    else if (patch_node["surface"]["type"].as<std::string>() == "Plane") {
      surfaces.push_back(Plane::from_yaml_node(patch_node["surface"]));
    }
    else {
      CROBOT_ASSERT_ERROR(false, "Unknown surface type");
    }

    math::Vector2d scale(1.0, 1.0);

    if (patch_node["uv_scale"].IsDefined())
      scale = math::Vector2d(
          patch_node["uv_scale"][0].as<double>(),
          patch_node["uv_scale"][1].as<double>());

    uv_scale.push_back(scale);

    bounds_u.push_back(math::Vector2d(
        patch_node["bounds_u"][0].as<double>(),
        patch_node["bounds_u"][1].as<double>()));
    bounds_v.push_back(math::Vector2d(
        patch_node["bounds_v"][0].as<double>(),
        patch_node["bounds_v"][1].as<double>()));
    bpos.push_back(math::Vector3d(
        patch_node["bpos"][0].as<double>(),
        patch_node["bpos"][1].as<double>(),
        patch_node["bpos"][2].as<double>()));
    rotm.push_back(math::Quaternion(
                       patch_node["quat"][0].as<double>(),
                       patch_node["quat"][1].as<double>(),
                       patch_node["quat"][2].as<double>(),
                       patch_node["quat"][3].as<double>())
                       .as_rotm());
  }

  return std::make_shared<SurfacePatchSwitch>(
      surfaces, bounds_u, bounds_v, bpos, rotm, uv_scale, idx_active);
}

//==============================================================================
bool SurfacePatchSwitch::patch_update(double u, double v) {
  (void)v;

  // update only if there are multiple surfaces
  if (m_surface_all.size() <= 1)
    return false;

  // switch to the next surface if u or v reaches the boundary
  if (u <= m_bounds_all[m_idx_active].first[0] * 0.95
      || u >= m_bounds_all[m_idx_active].first[1] * 0.95) {
    switch_to_next();
    return true;
  }

  return false;
}

//==============================================================================
void SurfacePatchSwitch::switch_to_next() {

  m_idx_active = (m_idx_active + 1) % m_surface_all.size();
  // CROBOT_LOG_INFO(
  //     "Switching to the next surface patch. active id = {}", m_idx_active);
  update(
      m_surface_all[m_idx_active],
      m_bounds_all[m_idx_active].first,
      m_bounds_all[m_idx_active].second,
      m_bpos_all[m_idx_active],
      m_rotm_all[m_idx_active],
      m_uv_scale_all[m_idx_active]);
}

//==============================================================================
SurfacePatchSwitch::Ptr SurfacePatchSwitch::clone() {
  return std::make_shared<SurfacePatchSwitch>(
      m_surface_all,
      m_bounds_all,
      m_bpos_all,
      m_rotm_all,
      m_uv_scale_all,
      m_idx_active);
}

//==============================================================================
YAML::Node SurfacePatchSwitch::to_yaml_node() const {
  YAML::Node node;
  node["type"]            = "SurfacePatchSwitch";
  node["active_patch_id"] = m_idx_active;
  for (size_t i = 0; i < m_surface_all.size(); ++i) {
    YAML::Node patch_node;
    patch_node["surface"] = m_surface_all[i]->to_yaml_node();

    // math::Vector2d scale = m_uv_scale_all[i];

    // scale it back
    patch_node["bounds_u"] = std::vector<double>{m_bounds_all[i].first[0],
                                                 m_bounds_all[i].first[1]};

    patch_node["bounds_v"] = std::vector<double>{m_bounds_all[i].second[0],
                                                 m_bounds_all[i].second[1]};

    patch_node["bpos"] = std::vector<double>{
        m_bpos_all[i](0), m_bpos_all[i](1), m_bpos_all[i](2)};
    math::Quaternion quat(m_rotm_all[i]);
    patch_node["quat"]
        = std::vector<double>{quat.w(), quat.x(), quat.y(), quat.z()};
    patch_node["uv_scale"]
        = std::vector<double>{m_uv_scale_all[i](0), m_uv_scale_all[i](1)};
    node["patches"].push_back(patch_node);
  }
  return node;
}

//==============================================================================
math::Vector2d SurfacePatchSwitch::get_uv_scale() const {
  return m_uv_scale_all[m_idx_active];
}
