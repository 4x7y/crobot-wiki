#pragma once

#include "crobot/math/surface.hpp"

namespace crobot::math {

class Sphere : public Surface {
public:
  Sphere(double radius);

  static std::shared_ptr<const Sphere> from_yaml_node(const YAML::Node& node);

  /**
   * @brief The parametric representation of a sphere.
   *
   * The parametric equations of an sphere can be written as follows:
   *
   * x = r * cos(u) * sin(v)
   * y = r * sin(u) * sin(v)
   * z = r * cos(v)
   *
   * @param u The parametric variable u in the range of [0, 2*pi)
   * @param v The parametric variable v in the range of [0, pi]
   * @return Vector3d The position of the point on the surface
   */
  Vector3d evaluate(double u, double v) const override;

  // Documentation inherited
  Vector3d normal(double u, double v) const override;

  // Documentation inherited
  array2<Vector3d> derivatives(int num_ders, double u, double v) const override;

  // Documentation inherited
  Matrix32 normal_jacb(double u, double v) const override;

  YAML::Node to_yaml_node() const override;

private:
  double m_radius;
};

} // namespace crobot::math

