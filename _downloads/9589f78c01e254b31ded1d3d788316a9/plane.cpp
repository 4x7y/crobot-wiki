#include "crobot/math/plane.hpp"

#include "crobot/common/common.hpp"

using namespace crobot;
using namespace math;

std::shared_ptr<const Plane> Plane::from_yaml_node(const YAML::Node& node) {
  CROBOT_ASSERT_ERROR(
      node["type"].as<std::string>() == "Plane", "type is not plane");

  return std::make_shared<const Plane>();
}

//==============================================================================
Vector3d Plane::evaluate(double u, double v) const {
  return math::Vector3d(u, v, 0);
}

//==============================================================================
array2<Vector3d> Plane::derivatives(
    int num_ders, double /* u */, double /* v */) const {
  array2<Vector3d> deriv
      = array2<Vector3d>(num_ders, num_ders, Vector3d::Zero());

  if (num_ders >= 1) {
    deriv(1, 0) = Vector3d(
        1,  // dxdu
        0,  // dydu
        0); // dzdu

    deriv(0, 1) = Vector3d(
        0,  // dxdv
        1,  // dydv
        0); // dzdv
  }

  return deriv;
}

//==============================================================================
YAML::Node Plane::to_yaml_node() const {
  YAML::Node node;
  node["type"] = "Plane";
  return node;
}
