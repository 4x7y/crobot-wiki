#include "crobot/math/rational_curve.hpp"

#include "crobot/math/bspline_curve.hpp"
#include "crobot/math/spline.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
RationalCurve::RationalCurve(const BSplineCurve& crv)
  : RationalCurve(crv, std::vector<double>(control_points.size(), 1.0)) {
}

//==============================================================================
RationalCurve::RationalCurve(
    const BSplineCurve& crv, const std::vector<double>& weights)
  : RationalCurve(crv.degree, crv.knots, crv.control_points, weights) {
}

//==============================================================================
RationalCurve::RationalCurve(
    std::size_t degree,
    const std::vector<double>& knots,
    const std::vector<Vector3d>& control_points,
    const std::vector<double> weights)
  : degree(degree),
    knots(knots),
    control_points(control_points),
    weights(weights) {
}

//==============================================================================
Vector3d RationalCurve::evaluate(double u) const {
  // Compute homogenous coordinates of control points
  std::vector<Vector4d> Cw;
  Cw.reserve(control_points.size());
  for (std::size_t i = 0; i < control_points.size(); i++) {
    Cw.push_back(control_points[i].cart2homo(weights[i]));
  }

  // Compute point using homogenous coordinates
  Vector4d pointw = internal::evaluate(degree, knots, Cw, u);

  // Convert back to cartesian coordinates
  return pointw.homo2cart();
}

//==============================================================================
std::vector<Vector3d> RationalCurve::derivatives(int num_ders, double u) const {
  std::vector<Vector3d> curve_ders;
  curve_ders.reserve(num_ders + 1);

  // Compute homogenous coordinates of control points
  std::vector<Vector4d> Cw;
  Cw.reserve(control_points.size());
  for (std::size_t i = 0; i < control_points.size(); i++) {
    Cw.push_back(control_points[i].cart2homo(weights[i]));
  }

  // Derivatives of Cw
  std::vector<Vector4d> Cwders
      = internal::derivatives(degree, knots, Cw, num_ders, u);

  // Split Cwders into coordinates and weights
  std::vector<Vector3d> Aders;
  std::vector<double> wders;
  for (const auto& val : Cwders) {
    Aders.push_back(val.homo2cart_truncate());
    wders.push_back(val.w());
  }

  // Compute rational derivatives
  for (int k = 0; k <= num_ders; k++) {
    Vector3d v = Aders[k];
    for (int i = 1; i <= k; i++) {
      v -= internal::binomial(k, i) * wders[i] * curve_ders[k - i];
    }
    curve_ders.push_back(v / wders[0]);
  }
  return curve_ders;
}

//==============================================================================
Vector3d RationalCurve::tangent(double u) const {
  std::vector<Vector3d> ders = derivatives(1, u);
  Vector3d du                = ders[1];
  double du_len              = du.norm();
  if (!is_zero(du_len)) {
    du /= du_len;
  }
  return du;
}

//==============================================================================
bool RationalCurve::is_valid() const {
  return internal::is_curve_valid(degree, knots, control_points, weights);
}

//==============================================================================
bool RationalCurve::is_closed() const {
  return internal::is_1d_array_closed(degree, control_points)
         && internal::is_1d_array_closed(degree, weights)
         && internal::is_knot_vector_closed(degree, knots);
}

//==============================================================================
RationalCurve RationalCurve::insert_knot(double u, int repeat) const {
  RationalCurve new_crv;
  new_crv.degree = degree;

  // Convert to homogenous coordinates
  std::vector<Vector4d> Cw;
  Cw.reserve(control_points.size());
  for (std::size_t i = 0; i < control_points.size(); ++i) {
    Cw.push_back(control_points[i].cart2homo(weights[i]));
  }

  // Perform knot insertion and get new knots and control points
  std::vector<Vector4d> new_Cw;
  std::vector<double> new_knots;
  internal::insert_curve_knot(
      degree, knots, Cw, u, repeat, new_crv.knots, new_Cw);

  // Convert back to cartesian coordinates
  new_crv.control_points.reserve(new_Cw.size());
  new_crv.weights.reserve(new_Cw.size());
  for (std::size_t i = 0; i < new_Cw.size(); ++i) {
    new_crv.control_points.push_back(new_Cw[i].homo2cart());
    new_crv.weights.push_back(new_Cw[i].w());
  }
  return new_crv;
}

//==============================================================================
std::tuple<RationalCurve, RationalCurve> RationalCurve::split(double u) const {
  RationalCurve left, right;
  left.degree  = degree;
  right.degree = degree;

  std::vector<Vector4d> Cw, left_Cw, right_Cw;
  Cw.reserve(control_points.size());
  for (std::size_t i = 0; i < control_points.size(); ++i) {
    Cw.push_back(control_points[i].cart2homo(weights[i]));
  }

  internal::split_curve(
      degree, knots, Cw, u, left.knots, left_Cw, right.knots, right_Cw);

  left.control_points.reserve(left_Cw.size());
  left.weights.reserve(left_Cw.size());
  right.control_points.reserve(right_Cw.size());
  right.weights.reserve(right_Cw.size());
  for (std::size_t i = 0; i < left_Cw.size(); ++i) {
    left.control_points.push_back(left_Cw[i].homo2cart());
    left.weights.push_back(left_Cw[i].w());
  }
  for (std::size_t i = 0; i < right_Cw.size(); ++i) {
    right.control_points.push_back(right_Cw[i].homo2cart());
    right.weights.push_back(right_Cw[i].w());
  }
  return std::make_tuple(std::move(left), std::move(right));
}
