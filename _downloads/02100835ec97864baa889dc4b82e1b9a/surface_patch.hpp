#pragma once

#include <memory>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "crobot/math/quaternion.hpp"
#include "crobot/math/surface.hpp"

namespace crobot::math {

class SurfacePatch {
public:
  using Ptr      = std::shared_ptr<SurfacePatch>;
  using ConstPtr = std::shared_ptr<const SurfacePatch>;

  /**
   * @brief Construct a new Surface Patch object
   *
   * @param surface surface object
   * @param bounds_u bounds u of surface parametrarization (no scaling)
   * @param bounds_v bounds v of surface parametrarization (no scaling)
   * @param bpos position in the body frame
   * @param rotm rotation matrix in the body frame
   * @param uv_scale scaling factor for uv
   */
  SurfacePatch(
      Surface::ConstPtr surface,
      const math::Vector2d& bounds_u,
      const math::Vector2d& bounds_v,
      const math::Vector3d& bpos     = math::Vector3d::Zero(),
      const math::Matrix3d& rotm     = math::Matrix3d::Identity(),
      const math::Vector2d& uv_scale = math::Vector2d::Ones());

  SurfacePatch(Surface::ConstPtr surface, YAML::Node node);

  math::Vector3d evaluate(const math::Vector2d& uv) const;

  math::Matrix32 jacb(const math::Vector2d& uv) const;

  math::Matrix6X hess(const math::Vector2d& uv) const;

  array2<math::Vector3d> derivatives(
      int degree, const math::Vector2d& uv) const;

  math::Matrix32 jacr(const math::Vector2d& uv) const;

  math::Vector3d normal(const math::Vector2d& uv) const;

  math::Matrix32 normal_jacb(const math::Vector2d& uv) const;

  math::Matrix6X normal_hess(const math::Vector2d& uv) const;

  const math::Vector2d& get_lb() const;

  const math::Vector2d& get_ub() const;

  math::Vector2d get_u_bounds() const;

  math::Vector2d get_v_bounds() const;

  math::Matrix4d get_frame(const math::Vector2d& uv) const;

  bool is_boundary(const math::Vector2d& uv) const;

  void update(
      Surface::ConstPtr surface,
      const math::Vector2d& bounds_u,
      const math::Vector2d& bounds_v,
      const math::Vector3d& bpos     = math::Vector3d::Zero(),
      const math::Matrix3d& rotm     = math::Matrix3d::Identity(),
      const math::Vector2d& uv_scale = math::Vector2d::Ones());

  // used for ICP solver
  void rotate(const math::Matrix3d& rotm);

  void translate(const math::Vector3d& bpos);

  math::Matrix4d get_tform() const;

  void set_bpos(const math::Vector3d& bpos);

  void set_rotm(const math::Matrix3d& rotm);

private:
  std::shared_ptr<const Surface> m_surface;
  std::pair<math::Vector2d, math::Vector2d> m_bounds;
  math::Vector3d m_bpos;
  math::Matrix3d m_rotm;
  math::Vector2d m_uv_scale;
};

} // namespace crobot::math

