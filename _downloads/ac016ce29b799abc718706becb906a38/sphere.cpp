#include "crobot/math/sphere.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
Sphere::Sphere(double radius) : Surface(), m_radius(radius) {
}

//==============================================================================
std::shared_ptr<const Sphere> Sphere::from_yaml_node(const YAML::Node& node) {
  CROBOT_ASSERT_ERROR(
      node["type"].as<std::string>() == "Sphere", "type is not sphere");

  return std::make_shared<const Sphere>(node["size"].as<double>());
}

//==============================================================================
Vector3d Sphere::evaluate(double u, double v) const {
  Vector3d bpos = Vector3d(
      m_radius * cos(u) * sin(v),
      m_radius * sin(u) * sin(v),
      m_radius * cos(v));
  return bpos;
}

//==============================================================================
Vector3d Sphere::normal(double u, double v) const {
  Vector3d bvec = Vector3d(
      cos(u) * sin(v), //
      sin(u) * sin(v), //
      cos(v));
  return bvec;
}

//==============================================================================
Matrix32 Sphere::normal_jacb(double u, double v) const {
  Matrix32 jacb;
  jacb.col(0) = Vector3d(
      -sin(u) * sin(v), //
      cos(u) * sin(v),  //
      0);
  jacb.col(1) = Vector3d(
      cos(u) * cos(v), //
      sin(u) * cos(v), //
      -sin(v));
  return jacb;
}

//==============================================================================
array2<Vector3d> Sphere::derivatives(int num_ders, double u, double v) const {
  CROBOT_ASSERT_ERROR(num_ders <= 2, "num_ders must be <= 2.");
  array2<Vector3d> deriv
      = array2<Vector3d>(num_ders + 1, num_ders + 1, Vector3d::Zero());

  if (num_ders >= 1) {
    deriv(1, 0) = Vector3d(
        -m_radius * sin(u) * sin(v), // dxdu
        +m_radius * cos(u) * sin(v), // dydu
        0);                          // dzdu

    deriv(0, 1) = Vector3d(
        +m_radius * cos(u) * cos(v), // dxdv
        +m_radius * sin(u) * cos(v), // dydv
        -m_radius * sin(v));         // dzdv
  }

  if (num_ders >= 2) {
    deriv(2, 0) = Vector3d(
        -m_radius * cos(u) * sin(v), // d2xdudu
        -m_radius * sin(u) * sin(v), // d2ydudu
        0);                          // d2zdudu

    deriv(0, 2) = Vector3d(
        -m_radius * cos(u) * sin(v), // d2xdudv
        -m_radius * sin(u) * sin(v), // d2ydudv
        0);                          // d2zdudv

    deriv(1, 1) = Vector3d(
        -m_radius * cos(v) * sin(u), // d2xdudv
        +m_radius * cos(u) * cos(v), // d2ydudv
        0);                          // d2zdudv
  }

  return deriv;
}

//==============================================================================
YAML::Node Sphere::to_yaml_node() const {
  YAML::Node node;
  node["type"] = "Sphere";
  node["size"] = m_radius;
  return node;
}
