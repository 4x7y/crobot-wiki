#pragma once

#include "crobot/math/array.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/surface.hpp"

namespace crobot::math {

class Plane : public Surface {
public:
  Plane() = default;

  static std::shared_ptr<const Plane> from_yaml_node(const YAML::Node& node);

  Vector3d evaluate(double u, double v) const override;

  array2<Vector3d> derivatives(int num_ders, double u, double v) const override;

  YAML::Node to_yaml_node() const override;

private:
};

} // namespace crobot::math

