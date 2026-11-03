#pragma once

#include <memory>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "crobot/math/curve.hpp"
#include "crobot/math/quaternion.hpp"

namespace crobot::math {

class CurveSegment {
public:
  CurveSegment(
      Curve::ConstPtr curve,
      const math::Vector2d& bounds_u,
      const math::Vector3d& bpos = math::Vector3d::Zero(),
      const math::Matrix3d& rotm = math::Matrix3d::Identity());

  CurveSegment(Curve::ConstPtr curve, YAML::Node node);

  math::Vector3d evaluate(double u) const;

  math::Vector3d jacb(double u) const;

  math::Vector3d tangent(double u) const;

  math::Vector3d tangent_jacb(double u) const;

  double get_lb() const;

  double get_ub() const;

private:
  std::shared_ptr<const Curve> m_curve;
  std::pair<double, double> m_bounds;
  math::Vector3d m_bpos;
  math::Matrix3d m_rotm;
};

} // namespace crobot::math

