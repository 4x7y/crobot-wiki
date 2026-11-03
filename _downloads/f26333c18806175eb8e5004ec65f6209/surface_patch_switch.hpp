#pragma once

#include <memory>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "crobot/math/surface_patch.hpp"

namespace crobot::math {

class SurfacePatchSwitch : public SurfacePatch {
public:
  using Ptr      = std::shared_ptr<SurfacePatchSwitch>;
  using ConstPtr = std::shared_ptr<const SurfacePatchSwitch>;

  SurfacePatchSwitch(
      const std::vector<Surface::ConstPtr>& surfaces,
      const std::vector<math::Vector2d>& bounds_u,
      const std::vector<math::Vector2d>& bounds_v,
      const std::vector<math::Vector3d>& bpos,
      const std::vector<math::Matrix3d>& rotm,
      const std::vector<math::Vector2d>& uv_scale
      = std::vector<math::Vector2d>({math::Vector2d::Ones()}),
      std::size_t active_patch_id = 0);

  SurfacePatchSwitch(
      const std::vector<Surface::ConstPtr>& surfaces,
      const std::vector<std::pair<math::Vector2d, math::Vector2d>>& bounds,
      const std::vector<math::Vector3d>& bpos,
      const std::vector<math::Matrix3d>& rotm,
      const std::vector<math::Vector2d>& uv_scale
      = std::vector<math::Vector2d>({math::Vector2d::Ones()}),
      std::size_t active_patch_id = 0);

  static std::shared_ptr<SurfacePatchSwitch> from_yaml_node(
      const YAML::Node& node);

  // check if u and v reaches the boundary, if so, switch to the next surface
  bool patch_update(double u, double v);

  void switch_to_next();

  SurfacePatchSwitch::Ptr clone();

  YAML::Node to_yaml_node() const;

  math::Vector2d get_uv_scale() const;

private:
  std::vector<Surface::ConstPtr> m_surface_all;
  std::vector<std::pair<math::Vector2d, math::Vector2d>> m_bounds_all;
  std::vector<math::Vector3d> m_bpos_all;
  std::vector<math::Matrix3d> m_rotm_all;
  std::vector<math::Vector2d> m_uv_scale_all;
  std::size_t m_idx_active;
};

} // namespace crobot::math

