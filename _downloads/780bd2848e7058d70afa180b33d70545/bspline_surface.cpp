#include "crobot/math/bspline_surface.hpp"

#include <yaml-cpp/yaml.h>

#include "crobot/common/assert.hpp"
#include "crobot/math/rational_surface.hpp"
#include "crobot/math/spline.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
BSplineSurface::BSplineSurface(const RationalSurface& srf)
  : degree_u(srf.degree_u),
    degree_v(srf.degree_v),
    knots_u(srf.knots_u),
    knots_v(srf.knots_v),
    control_points(srf.control_points) {
  m_filename = "";
}

//==============================================================================
BSplineSurface::BSplineSurface(
    std::size_t degree_u,
    std::size_t degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    array2<Vector3d> control_points)
  : degree_u(degree_u),
    degree_v(degree_v),
    knots_u(knots_u),
    knots_v(knots_v),
    control_points(control_points) {
  m_filename = "";
}

//==============================================================================
std::shared_ptr<const BSplineSurface> BSplineSurface::from_yaml_node(
    const YAML::Node& node) {
  CROBOT_ASSERT_ERROR(
      node["type"].as<std::string>() == "BSplineSurface",
      "type is not BSplineSurface");

  return std::make_shared<const BSplineSurface>(
      node["filename"].as<std::string>());
}

//==============================================================================
BSplineSurface::BSplineSurface(const std::string& filename) {
  m_filename      = filename;
  YAML::Node node = YAML::LoadFile(filename);
  CROBOT_ASSERT_ERROR(node.IsDefined(), "File not found: %s", filename.c_str());

  CROBOT_ASSERT_ERROR(node["degree_u"].IsDefined(), "degree_u not found");
  degree_u = node["degree_u"].as<std::size_t>();
  CROBOT_ASSERT_ERROR(node["degree_v"].IsDefined(), "degree_v not found");
  degree_v = node["degree_v"].as<std::size_t>();

  CROBOT_ASSERT_ERROR(node["knots_u"].IsDefined(), "knots_u not found");
  knots_u = node["knots_u"].as<std::vector<double>>();
  CROBOT_ASSERT_ERROR(node["knots_v"].IsDefined(), "knots_v not found");
  knots_v = node["knots_v"].as<std::vector<double>>();

  // m = n + p + 1
  // num_control_points = n + 1
  // degree: p
  // num_knots: m + 1

  // n + 1 = (m + 1) - p - 1
  int mu      = static_cast<int>(knots_u.size()) - 1;
  int mv      = static_cast<int>(knots_v.size()) - 1;
  int pu      = static_cast<int>(degree_u);
  int pv      = static_cast<int>(degree_v);
  int nu      = mu - pu - 1;
  int nv      = mv - pv - 1;
  int ncpts_u = nu + 1;
  int ncpts_v = nv + 1;

  CROBOT_ASSERT_ERROR(
      node["control_points"].IsDefined(), "control_points not found");
  YAML::Node node_cp = node["control_points"];
  control_points.resize(ncpts_u, ncpts_v);
  CROBOT_ASSERT_ERROR(
      static_cast<int>(node_cp.size()) == ncpts_u,
      "control_points_u has wrong size. Expected %d, got %d.",
      ncpts_u,
      static_cast<int>(node_cp.size()));
  for (int i = 0; i < ncpts_u; ++i) {
    CROBOT_ASSERT_ERROR(
        static_cast<int>(node_cp[i].size()) == ncpts_v,
        "control_points_v has wrong size. Expected %d, got %d.",
        ncpts_v,
        static_cast<int>(node_cp[i].size()));
    for (int j = 0; j < ncpts_v; ++j) {
      std::vector<double> pt = node_cp[i][j].as<std::vector<double>>();
      control_points(i, j)   = Vector3d(pt[0], pt[1], pt[2]);
    }
  }
}

//==============================================================================
void BSplineSurface::scale(Vector3d scale) {
  for (int i = 0; i < control_points.rows(); ++i) {
    for (int j = 0; j < control_points.cols(); ++j) {
      control_points(i, j) = control_points(i, j).cwiseProduct(scale);
    }
  }
}

//==============================================================================
void BSplineSurface::translate(Vector3d vec) {
  for (int i = 0; i < control_points.rows(); ++i) {
    for (int j = 0; j < control_points.cols(); ++j) {
      control_points(i, j) = control_points(i, j) + vec;
    }
  }
}

//==============================================================================
Vector3d BSplineSurface::evaluate(double u, double v) const {
  return internal::evaluate(
      degree_u, degree_v, knots_u, knots_v, control_points, u, v);
}

//==============================================================================
array2<Vector3d> BSplineSurface::derivatives(
    int num_ders, double u, double v) const {
  return internal::derivatives(
      degree_u, degree_v, knots_u, knots_v, control_points, num_ders, u, v);
}

//==============================================================================
std::tuple<Vector3d, Vector3d> BSplineSurface::tangent(
    double u, double v) const {
  array2<Vector3d> ptder = derivatives(1, u, v);
  Vector3d du            = ptder(1, 0);
  Vector3d dv            = ptder(0, 1);
  double du_len          = ptder(1, 0).norm();
  double dv_len          = ptder(0, 1).norm();
  if (!is_zero(du_len)) {
    du /= du_len;
  }
  if (!is_zero(dv_len)) {
    dv /= dv_len;
  }
  return std::make_tuple(std::move(du), std::move(dv));
}

//==============================================================================
Vector3d BSplineSurface::normal(double u, double v) const {
  array2<Vector3d> ptder = derivatives(1, u, v);
  Vector3d n             = ptder(0, 1).cross(ptder(1, 0));
  double n_len           = n.norm();
  if (!is_zero(n_len)) {
    n /= n_len;
  }
  return n;
}

//==============================================================================
bool BSplineSurface::is_valid() const {
  return internal::is_surface_valid(
      degree_u, degree_v, knots_u, knots_v, control_points);
}

//==============================================================================
bool BSplineSurface::is_closed_along_u() const {
  return internal::is_2d_array_closed_u(degree_u, control_points)
         && internal::is_knot_vector_closed(degree_u, knots_u);
}

//==============================================================================
bool BSplineSurface::is_closed_along_v() const {
  return internal::is_2d_array_closed_v(degree_v, control_points)
         && internal::is_knot_vector_closed(degree_v, knots_v);
}

//==============================================================================
BSplineSurface BSplineSurface::insert_knot_along_u(double u, int repeat) const {
  BSplineSurface new_srf;
  new_srf.degree_u = degree_u;
  new_srf.degree_v = degree_v;
  new_srf.knots_v  = knots_v;
  internal::surfaceKnotInsert(
      new_srf.degree_u,
      knots_u,
      control_points,
      u,
      repeat,
      true,
      new_srf.knots_u,
      new_srf.control_points);
  return new_srf;
}

//==============================================================================
BSplineSurface BSplineSurface::insert_knot_along_v(double v, int repeat) const {
  BSplineSurface new_srf;
  new_srf.degree_u = degree_u;
  new_srf.degree_v = degree_v;
  new_srf.knots_u  = knots_u;
  // New knots and new control points after knot insertion
  internal::surfaceKnotInsert(
      degree_u,
      knots_u,
      control_points,
      v,
      repeat,
      false,
      new_srf.knots_v,
      new_srf.control_points);
  return new_srf;
}

//==============================================================================
std::tuple<BSplineSurface, BSplineSurface> BSplineSurface::split_along_u(
    double u) const {
  BSplineSurface left, right;
  left.degree_u  = degree_u;
  left.degree_v  = degree_v;
  left.knots_v   = knots_v;
  right.degree_u = degree_u;
  right.degree_v = degree_v;
  right.knots_v  = knots_v;
  internal::surfaceSplit(
      degree_u,
      knots_u,
      control_points,
      u,
      true,
      left.knots_u,
      left.control_points,
      right.knots_u,
      right.control_points);
  return std::make_tuple(std::move(left), std::move(right));
}

//==============================================================================
std::tuple<BSplineSurface, BSplineSurface> BSplineSurface::split_along_v(
    double v) const {
  BSplineSurface left, right;
  left.degree_u  = degree_u;
  left.degree_v  = degree_v;
  left.knots_u   = knots_u;
  right.degree_u = degree_u;
  right.degree_v = degree_v;
  right.knots_u  = knots_u;
  internal::surfaceSplit(
      degree_v,
      knots_v,
      control_points,
      v,
      false,
      left.knots_v,
      left.control_points,
      right.knots_v,
      right.control_points);
  return std::make_tuple(std::move(left), std::move(right));
}

//==============================================================================
bool crobot::math::internal::is_surface_valid(
    int degree_u,
    int degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<Vector3d>& control_points) {
  if (degree_u < 1 || degree_u > 9 || degree_v < 1 || degree_v > 9) {
    return false;
  }
  if (!is_valid_relation(degree_u, knots_u.size(), control_points.rows())
      || !is_valid_relation(degree_v, knots_v.size(), control_points.cols())) {
    return false;
  }
  if (!is_knot_vector_monotonic(knots_u)
      || !is_knot_vector_monotonic(knots_v)) {
    return false;
  }
  return true;
}

//==============================================================================
bool crobot::math::internal::is_surface_valid(
    int degree_u,
    int degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<Vector3d>& control_points,
    const array2<double>& weights) {
  if (!is_surface_valid(degree_u, degree_v, knots_u, knots_v, control_points)) {
    return false;
  }
  if (control_points.rows() != weights.rows()
      || control_points.cols() != weights.cols()) {
    return false;
  }
  return true;
}

//==============================================================================
YAML::Node BSplineSurface::to_yaml_node() const {
  YAML::Node node;
  node["type"]     = "BSplineSurface";
  node["filename"] = m_filename;
  return node;
}
