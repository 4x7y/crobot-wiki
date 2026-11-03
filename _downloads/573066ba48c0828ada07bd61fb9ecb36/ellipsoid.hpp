#pragma once

#include "crobot/math/surface.hpp"

namespace crobot::math {

class Ellipsoid : public Surface {
public:
  Ellipsoid(double ra, double rb, double rc);

  Ellipsoid(const Vector3d& size);

  static std::shared_ptr<const Ellipsoid> from_yaml_node(
      const YAML::Node& node);

  /**
   * @brief The parametric equations of an ellipsoid can be written as follows:
   *
   * x = a * cos(u) * sin(v)
   * y = b * sin(u) * sin(v)
   * z = c * cos(v)
   *
   * @param u The parametric variable u in the range of [0, 2*pi)
   * @param v The parametric variable v in the range of [0, pi]
   * @return Vector3d The position of the point on the surface
   */
  Vector3d evaluate(double u, double v) const override;

  // Documentation inherited
  Vector3d normal(double u, double v) const override;

  // Documentation inherited
  Matrix32 normal_jacb(double u, double v) const override;

  // Documentation inherited
  virtual array2<Vector3d> derivatives(
      int num_ders, double u, double v) const override;

  YAML::Node to_yaml_node() const override;

private:
  double m_ra;
  double m_rb;
  double m_rc;
};

} // namespace crobot::math

