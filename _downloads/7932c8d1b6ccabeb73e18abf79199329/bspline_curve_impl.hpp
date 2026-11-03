#pragma once

#include "crobot/math/bspline_curve.hpp"

namespace crobot::math::internal {

/**
 * @brief Returns whether the curve is valid.
 *
 * @param[in] degree Degree of curve
 * @param[in] knots Knot vector of curve
 * @param[in] control_points Control points of curve
 * @return Whether valid
 */
bool is_curve_valid(
    int degree,
    const std::vector<double>& knots,
    const std::vector<Vector3d>& control_points);

/**
 * Returns whether the curve is valid
 *
 * @param[in] degree Degree of curve
 * @param[in] knots Knot vector of curve
 * @param[in] control_points Control points of curve
 * @return Whether valid
 */
bool is_curve_valid(
    int degree,
    const std::vector<double>& knots,
    const std::vector<Vector3d>& control_points,
    const std::vector<double>& weights);

/**
 * @brief Returns whether the given knot vector is closed by checking the
 * periodicity of knot vectors near the start and end.
 *
 * @param[in] degree Degree of curve/surface
 * @param[in] vec Array of any control points/weights
 * @return Whether knot vector is closed
 */
template <typename T>
inline bool is_1d_array_closed(int degree, const std::vector<T>& vec) {
  for (int i = 0; i < degree; ++i) {
    int j = vec.size() + i - degree;
    if (length(vec[i] - vec[j]) > 1e-5) {
      return false;
    }
  }
  return true;
}

/**
 * Evaluate point on a nonrational NURBS curve
 * @param[in] degree Degree of the given curve.
 * @param[in] knots Knot vector of the curve.
 * @param[in] control_points Control points of the curve.
 * @param[in] u Parameter to evaluate the curve at.
 * @return point Resulting point on the curve at parameter u.
 */
template <int Dim>
VectorNd<Dim> evaluate(
    int degree,
    const std::vector<double>& knots,
    const std::vector<VectorNd<Dim>>& control_points,
    double u) {
  // Initialize result to 0s
  VectorNd<Dim> point = VectorNd<Dim>::Zero();

  // Find span and corresponding non-zero basis functions
  int span              = find_span(degree, knots, u);
  std::vector<double> N = bspline_basis(degree, span, knots, u);

  // Compute point
  for (int j = 0; j <= degree; j++) {
    point += N[j] * control_points[span - degree + j];
  }
  return point;
}

/**
 * @brief Evaluate derivatives of a non-rational NURBS curve
 *
 * @param[in] degree Degree of the curve
 * @param[in] knots Knot vector of the curve.
 * @param[in] control_points Control points of the curve.
 * @param[in] num_ders Number of times to derivate.
 * @param[in] u Parameter to evaluate the derivatives at.
 * @return curve_ders Derivatives of the curve at u.
 * E.g. curve_ders[n] is the nth derivative at u, where 0 <= n <= num_ders.
 */
template <int Dim>
std::vector<VectorNd<Dim>> derivatives(
    int degree,
    const std::vector<double>& knots,
    const std::vector<VectorNd<Dim>>& control_points,
    int num_ders,
    double u) {

  typedef VectorNd<Dim> Vector3d;
  using std::vector;

  std::vector<VectorNd<Dim>> curve_ders;
  curve_ders.resize(num_ders + 1);

  // Assign higher order derivatives to zero
  for (int k = degree + 1; k <= num_ders; k++) {
    curve_ders[k] = Vector3d::Zero();
  }

  // Find the span and corresponding non-zero basis functions & derivatives
  int span            = find_span(degree, knots, u);
  array2<double> ders = bspline_der_basis(degree, span, knots, u, num_ders);

  // Compute first num_ders derivatives
  int du = num_ders < degree ? num_ders : degree;
  for (int k = 0; k <= du; k++) {
    curve_ders[k] = Vector3d::Zero();
    for (int j = 0; j <= degree; j++) {
      curve_ders[k] += ders(k, j) * control_points[span - degree + j];
    }
  }
  return curve_ders;
}

/**
 * Insert knots in the curve
 * @param[in] deg Degree of the curve
 * @param[in] knots Knot vector of the curve
 * @param[in] cp Control points of the curve
 * @param[in] u Parameter to insert knot(s) at
 * @param[in] r Number of times to insert knot
 * @param[out] new_knots Updated knot vector
 * @param[out] new_cp Updated control points
 */
template <int Dim>
void insert_curve_knot(
    int deg,
    const std::vector<double>& knots,
    const std::vector<VectorNd<Dim>>& cp,
    double u,
    int r,
    std::vector<double>& new_knots,
    std::vector<VectorNd<Dim>>& new_cp) {
  int k = find_span(deg, knots, u);
  int s = knot_multiplicity(knots, k);
  if (s == deg) {
    return;
  }
  if ((r + s) > deg) {
    r = deg - s;
  }

  // Insert new knots between span and (span + 1)
  new_knots.resize(knots.size() + r);
  for (int i = 0; i < k + 1; ++i) {
    new_knots[i] = knots[i];
  }
  for (int i = 1; i < r + 1; ++i) {
    new_knots[k + i] = u;
  }
  for (std::size_t i = k + 1; i < knots.size(); ++i) {
    new_knots[i + r] = knots[i];
  }
  // Copy unaffected control points
  new_cp.resize(cp.size() + r);
  for (int i = 0; i < k - deg + 1; ++i) {
    new_cp[i] = cp[i];
  }
  for (int i = k - s; i < int(cp.size()); ++i) {
    new_cp[i + r] = cp[i];
  }
  // Copy affected control points
  std::vector<VectorNd<Dim>> tmp;
  tmp.resize(deg - s + 1);
  for (int i = 0; i < deg - s + 1; ++i) {
    tmp[i] = cp[k - deg + i];
  }
  // Modify affected control points
  for (int j = 1; j <= r; ++j) {
    int L = k - deg + j;
    for (int i = 0; i < deg - j - s + 1; ++i) {
      double a = (u - knots[L + i]) / (knots[i + k + 1] - knots[L + i]);
      tmp[i]   = (1 - a) * tmp[i] + a * tmp[i + 1];
    }
    new_cp[L]             = tmp[0];
    new_cp[k + r - j - s] = tmp[deg - j - s];
  }
  int L = k - deg + r;
  for (int i = L + 1; i < k - s; ++i) {
    new_cp[i] = tmp[i - L];
  }
}

/**
 * Split the curve into two
 * @param[in] degree Degree of curve
 * @param[in] knots Knot vector
 * @param[in] control_points Array of control points
 * @param[in] u Parameter to split curve
 * @param[out] left_knots Knots of the left part of the curve
 * @param[out] left_control_points Control points of the left part of the
 * curve
 * @param[out] right_knots Knots of the right part of the curve
 * @param[out] right_control_points Control points of the right part of the
 * curve
 */
template <int Dim>
void split_curve(
    int degree,
    const std::vector<double>& knots,
    const std::vector<VectorNd<Dim>>& control_points,
    double u,
    std::vector<double>& left_knots,
    std::vector<VectorNd<Dim>>& left_control_points,
    std::vector<double>& right_knots,
    std::vector<VectorNd<Dim>>& right_control_points) {
  std::vector<double> tmp_knots;
  std::vector<VectorNd<Dim>> tmp_cp;

  int span = find_span(degree, knots, u);
  int r    = degree - knot_multiplicity(knots, span);

  internal::insert_curve_knot(
      degree, knots, control_points, u, r, tmp_knots, tmp_cp);

  left_knots.clear();
  right_knots.clear();
  left_control_points.clear();
  right_control_points.clear();

  int span_l = find_span(degree, tmp_knots, u) + 1;
  for (int i = 0; i < span_l; ++i) {
    left_knots.push_back(tmp_knots[i]);
  }
  left_knots.push_back(u);

  for (int i = 0; i < degree + 1; ++i) {
    right_knots.push_back(u);
  }
  for (std::size_t i = span_l; i < tmp_knots.size(); ++i) {
    right_knots.push_back(tmp_knots[i]);
  }

  int ks = span - degree + 1;
  for (int i = 0; i < ks + r; ++i) {
    left_control_points.push_back(tmp_cp[i]);
  }
  for (int i = ks + r - 1; i < int(tmp_cp.size()); ++i) {
    right_control_points.push_back(tmp_cp[i]);
  }
}

} // namespace crobot::math::internal

