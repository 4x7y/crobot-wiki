#include <yaml-cpp/yaml.h>

#include "crobot/math/bspline_curve.hpp"
#include "crobot/math/rational_curve.hpp"
#include "crobot/math/spline.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
BSplineCurve::BSplineCurve(const RationalCurve& crv)
  : BSplineCurve(crv.degree, crv.knots, crv.control_points) {
}

//==============================================================================
BSplineCurve::BSplineCurve(
    std::size_t degree,
    const std::vector<double>& knots,
    const std::vector<Vector3d>& control_points)
  : degree(degree), knots(knots), control_points(control_points) {
}

//==============================================================================
BSplineCurve::BSplineCurve(const std::string& filename) {
  YAML::Node node = YAML::LoadFile(filename);
  degree          = node["degree"].as<std::size_t>();
  knots           = node["knots"].as<std::vector<double>>();

  YAML::Node node_cp = node["control_points"];
  for (YAML::const_iterator it = node_cp.begin(); it != node_cp.end(); ++it) {
    std::vector<double> pt = it->as<std::vector<double>>();
    control_points.push_back(Vector3d(pt[0], pt[1], pt[2]));
  }
}

//==============================================================================
Vector3d BSplineCurve::evaluate(double u) const {
  return internal::evaluate(degree, knots, control_points, u);
}

//==============================================================================
std::vector<Vector3d> BSplineCurve::derivatives(int num_ders, double u) const {
  return internal::derivatives(degree, knots, control_points, num_ders, u);
}

//==============================================================================
Vector3d BSplineCurve::tangent(double u) const {
  std::vector<Vector3d> ders = derivatives(1, u);
  Vector3d du                = ders[1];
  double du_len              = du.norm();
  if (!is_zero(du_len)) {
    du /= du_len;
  }
  return du;
}

//==============================================================================
bool BSplineCurve::is_valid() const {
  return internal::is_curve_valid(degree, knots, control_points);
}

//==============================================================================
bool BSplineCurve::is_closed() const {
  return internal::is_1d_array_closed(degree, control_points)
         && internal::is_knot_vector_closed(degree, knots);
}

//==============================================================================
BSplineCurve BSplineCurve::insert_knot(double u, int repeat) const {
  BSplineCurve new_crv;
  new_crv.degree = degree;
  internal::insert_curve_knot(
      degree,
      knots,
      control_points,
      u,
      repeat,
      new_crv.knots,
      new_crv.control_points);
  return new_crv;
}

//==============================================================================
std::tuple<BSplineCurve, BSplineCurve> BSplineCurve::split(double u) const {
  BSplineCurve left, right;
  left.degree  = degree;
  right.degree = degree;
  internal::split_curve(
      degree,
      knots,
      control_points,
      u,
      left.knots,
      left.control_points,
      right.knots,
      right.control_points);
  return std::make_tuple(std::move(left), std::move(right));
}

//==============================================================================
bool crobot::math::internal::is_curve_valid(
    int degree,
    const std::vector<double>& knots,
    const std::vector<Vector3d>& control_points) {
  if (degree < 1 || degree > 9) {
    return false;
  }
  if (!is_valid_relation(degree, knots.size(), control_points.size())) {
    return false;
  }
  if (!is_knot_vector_monotonic(knots)) {
    return false;
  }
  return true;
}

//==============================================================================
bool crobot::math::internal::is_curve_valid(
    int degree,
    const std::vector<double>& knots,
    const std::vector<Vector3d>& control_points,
    const std::vector<double>& weights) {
  if (!is_valid_relation(degree, knots.size(), control_points.size())) {
    return false;
  }
  if (weights.size() != control_points.size()) {
    return false;
  }
  return true;
}
