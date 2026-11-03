#include "crobot/math/ellipsoid.hpp"

#include "crobot/common/common.hpp"
#include "crobot/math/array.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
Ellipsoid::Ellipsoid(double ra, double rb, double rc)
  : Surface(), m_ra(ra), m_rb(rb), m_rc(rc) {
}

//==============================================================================
Ellipsoid::Ellipsoid(const Vector3d& size)
  : Ellipsoid(size(0), size(1), size(2)) {
}

//==============================================================================
std::shared_ptr<const Ellipsoid> Ellipsoid::from_yaml_node(
    const YAML::Node& node) {
  CROBOT_ASSERT_ERROR(
      node["type"].as<std::string>() == "Ellipsoid", "type is not ellipsoid");

  return std::make_shared<const Ellipsoid>(
      node["ra"].as<double>(),
      node["rb"].as<double>(),
      node["rc"].as<double>());
}

//==============================================================================
Vector3d Ellipsoid::evaluate(double u, double v) const {
  Vector3d bpos = Vector3d(
      m_ra * cos(u) * sin(v), //
      m_rb * sin(u) * sin(v), //
      m_rc * cos(v));
  return bpos;
}

//==============================================================================
Vector3d Ellipsoid::normal(double u, double v) const {
  return Vector3d(
             cos(u) * sin(v) / m_ra, //
             sin(u) * sin(v) / m_rb, //
             cos(v) / m_rc)
      .normalized();
}

//==============================================================================
Matrix32 Ellipsoid::normal_jacb(double u, double v) const {
  math::Vector3d m = Vector3d(
      cos(u) * sin(v) / m_ra, //
      sin(u) * sin(v) / m_rb, //
      cos(v) / m_rc);
  double m_norm = m.norm();
  double m0     = m(0) / m_norm;
  double m1     = m(1) / m_norm;
  double m2     = m(2) / m_norm;

  Matrix3d jacb;
  jacb(0, 0) = -(-1.0 + m0 * m0) / m_norm;
  jacb(1, 0) = -(m0 * m1) / m_norm;
  jacb(2, 0) = -(m0 * m2) / m_norm;

  jacb(0, 1) = -(m0 * m1) / m_norm;
  jacb(1, 1) = -(-1.0 + m1 * m1) / m_norm;
  jacb(2, 1) = -(m1 * m2) / m_norm;

  jacb(0, 2) = -(m0 * m2) / m_norm;
  jacb(1, 2) = -(m1 * m2) / m_norm;
  jacb(2, 2) = -(-1.0 + m2 * m2) / m_norm;

  Matrix32 dmduv;
  dmduv.col(0) = Vector3d(
      -sin(u) * sin(v) / m_ra, //
      cos(u) * sin(v) / m_rb,  //
      0);
  dmduv.col(1) = Vector3d(
      cos(u) * cos(v) / m_ra, //
      sin(u) * cos(v) / m_rb, //
      -sin(v) / m_rc);

  return jacb * dmduv;
}

//==============================================================================
array2<Vector3d> Ellipsoid::derivatives(
    int num_ders, double u, double v) const {
  CROBOT_ASSERT_ERROR(num_ders <= 2, "num_ders must be <= 2.");
  array2<Vector3d> deriv
      = array2<Vector3d>(num_ders + 1, num_ders + 1, Vector3d::Zero());

  if (num_ders >= 1) {
    deriv(1, 0) = Vector3d(
        -m_ra * sin(u) * sin(v), // dxdu
        +m_rb * cos(u) * sin(v), // dydu
        0);                      // dzdu

    deriv(0, 1) = Vector3d(
        +m_ra * cos(u) * cos(v), // dxdv
        +m_rb * sin(u) * cos(v), // dydv
        -m_rc * sin(v));         // dzdv
  }

  if (num_ders >= 2) {
    deriv(2, 0) = Vector3d(
        -m_ra * cos(u) * sin(v), // d2xdudu
        -m_rb * sin(u) * sin(v), // d2ydudu
        0);                      // d2zdudu

    deriv(0, 2) = Vector3d(
        -m_ra * cos(u) * sin(v), // d2xdvdv
        -m_rb * sin(u) * sin(v), // d2ydvdv
        -m_rc * cos(v));         // d2zdvdv

    deriv(1, 1) = Vector3d(
        -m_ra * sin(u) * cos(v), // d2xdudv
        +m_rb * cos(u) * cos(v), // d2ydudv
        0);                      // d2zdudv
  }

  return deriv;
}

//==============================================================================
YAML::Node Ellipsoid::to_yaml_node() const {
  YAML::Node node;
  node["type"] = "Ellipsoid";
  node["ra"]   = m_ra;
  node["rb"]   = m_rb;
  node["rc"]   = m_rc;
  return node;
}
