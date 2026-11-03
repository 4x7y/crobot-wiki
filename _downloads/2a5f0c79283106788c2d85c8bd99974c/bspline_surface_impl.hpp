#pragma once

#include "crobot/math/bspline_surface.hpp"

namespace crobot::math::internal {

/**
 * Returns whether the surface is valid
 *
 * @param[in] degree_u Degree of surface along u-direction
 * @param[in] degree_v Degree of surface along v-direction
 * @param[in] knots_u Knot vector of surface along u-direction
 * @param[in] knots_v Knot vector of surface along v-direction
 * @param[in] control_points Control points grid of surface
 *
 * @return Whether valid
 */
bool is_surface_valid(
    int degree_u,
    int degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<Vector3d>& control_points);

/**
 * Returns whether the rational surface is valid
 *
 * @param[in] degree_u Degree of surface along u-direction
 * @param[in] degree_v Degree of surface along v-direction
 * @param[in] knots_u Knot vector of surface along u-direction
 * @param[in] knots_v Knot vector of surface along v-direction
 * @param[in] control_points Control points grid of surface
 * @param[in] weights Weights corresponding to control point grid of surface
 *
 * @return Whether valid
 */
bool is_surface_valid(
    int degree_u,
    int degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<Vector3d>& control_points,
    const array2<double>& weights);

/**
 * Returns whether the 2D array is closed along the u-direction
 * i.e., along rows.
 * @param[in] degree_u Degree along u-direction
 * @param[in] arr 2D array of control points / weights
 * @return Whether closed along u-direction
 */
template <typename T>
inline bool is_2d_array_closed_u(int degree_u, const array2<T>& arr) {
  for (int i = 0; i < degree_u; ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      int k = arr.cols() - degree_u + i;
      if (length(arr(i, j) - arr(k, j)) > 1e-5) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Returns whether the 2D array is closed along the v-direction i.e.,
 * along columns.
 *
 * @param[in] degree_v Degree along v-direction
 * @param[in] arr 2D array of control points / weights
 * @return Whether closed along v-direction
 */
template <typename T>
inline bool is_2d_array_closed_v(int degree_v, const array2<T>& arr) {
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < degree_v; j++) {
      int k = arr.rows() - degree_v + i;
      if (length(arr(i, j) - arr(i, k)) > 1e-5) {
        return false;
      }
    }
  }
  return true;
}

/**
 * Evaluate point on a nonrational NURBS surface
 *
 * @param[in] degree_u Degree of the given surface in u-direction.
 * @param[in] degree_v Degree of the given surface in v-direction.
 * @param[in] knots_u Knot vector of the surface in u-direction.
 * @param[in] knots_v Knot vector of the surface in v-direction.
 * @param[in] control_points Control points of the surface in a 2d array.
 * @param[in] u Parameter to evaluate the surface at.
 * @param[in] v Parameter to evaluate the surface at.
 * @return point Resulting point on the surface at (u, v).
 */
template <int Dim>
VectorNd<Dim> evaluate(
    int degree_u,
    int degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<VectorNd<Dim>>& control_points,
    double u,
    double v) {

  // Initialize result to 0s
  VectorNd<Dim> point = VectorNd<Dim>::Zero();

  // Find span and non-zero basis functions
  int span_u             = find_span(degree_u, knots_u, u);
  int span_v             = find_span(degree_v, knots_v, v);
  std::vector<double> Nu = bspline_basis(degree_u, span_u, knots_u, u);
  std::vector<double> Nv = bspline_basis(degree_v, span_v, knots_v, v);

  for (int l = 0; l <= degree_v; l++) {
    VectorNd<Dim> temp = VectorNd<Dim>::Zero();
    for (int k = 0; k <= degree_u; k++) {
      temp += Nu[k]
              * control_points(span_u - degree_u + k, span_v - degree_v + l);
    }
    point += Nv[l] * temp;
  }
  return point;
}

/**
 * Evaluate derivatives on a non-rational NURBS surface
 *
 * @param[in] degree_u Degree of the given surface in u-direction.
 * @param[in] degree_v Degree of the given surface in v-direction.
 * @param[in] knots_u Knot vector of the surface in u-direction.
 * @param[in] knots_v Knot vector of the surface in v-direction.
 * @param[in] control_points Control points of the surface in a 2D array.
 * @param[in] num_ders Number of times to differentiate
 * @param[in] u Parameter to evaluate the surface at.
 * @param[in] v Parameter to evaluate the surface at.
 * @param[out] surf_ders Derivatives of the surface at (u, v).
 */
template <int Dim>
array2<VectorNd<Dim>> derivatives(
    int degree_u,
    int degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<VectorNd<Dim>>& control_points,
    int num_ders,
    double u,
    double v) {

  array2<VectorNd<Dim>> surf_ders(
      num_ders + 1, num_ders + 1, VectorNd<Dim>::Zero());

  // Set higher order derivatives to 0
  for (int k = degree_u + 1; k <= num_ders; k++) {
    for (int l = degree_v + 1; l <= num_ders; l++) {
      surf_ders(k, l) = VectorNd<Dim>::Zero();
    }
  }

  // Find span and basis function derivatives
  int span_u = find_span(degree_u, knots_u, u);
  int span_v = find_span(degree_v, knots_v, v);
  array2<double> ders_u
      = bspline_der_basis(degree_u, span_u, knots_u, u, num_ders);
  array2<double> ders_v
      = bspline_der_basis(degree_v, span_v, knots_v, v, num_ders);

  // Number of non-zero derivatives is <= degree
  int du = std::min(num_ders, degree_u);
  int dv = std::min(num_ders, degree_v);

  std::vector<VectorNd<Dim>> temp;
  temp.resize(degree_v + 1);
  // Compute derivatives
  for (int k = 0; k <= du; k++) {
    for (int s = 0; s <= degree_v; s++) {
      temp[s] = VectorNd<Dim>::Zero();
      for (int r = 0; r <= degree_u; r++) {
        temp[s]
            += ders_u(k, r)
               * control_points(span_u - degree_u + r, span_v - degree_v + s);
      }
    }

    int dd = std::min(num_ders - k, dv);

    for (int l = 0; l <= dd; l++) {
      for (int s = 0; s <= degree_v; s++) {
        surf_ders(k, l) += ders_v(l, s) * temp[s];
      }
    }
  }
  return surf_ders;
}

/**
 * Insert knots in the surface along one direction
 * @param[in] degree Degree of the surface along which to insert knot
 * @param[in] knots Knot vector
 * @param[in] cp 2D array of control points
 * @param[in] knot Knot value to insert
 * @param[in] r Number of times to insert
 * @param[in] along_u Whether inserting along u-direction
 * @param[out] new_knots Updated knot vector
 * @param[out] new_cp Updated control points
 */
template <int Dim>
void surfaceKnotInsert(
    int degree,
    const std::vector<double>& knots,
    const array2<VectorNd<Dim>>& cp,
    double knot,
    int r,
    bool along_u,
    std::vector<double>& new_knots,
    array2<VectorNd<Dim>>& new_cp) {
  int span = find_span(degree, knots, knot);
  int s    = knot_multiplicity(knots, span);
  if (s == degree) {
    return;
  }
  if ((r + s) > degree) {
    r = degree - s;
  }

  // Create a new knot vector
  new_knots.resize(knots.size() + r);
  for (int i = 0; i <= span; ++i) {
    new_knots[i] = knots[i];
  }
  for (int i = 1; i <= r; ++i) {
    new_knots[span + i] = knot;
  }
  for (std::size_t i = span + 1; i < knots.size(); ++i) {
    new_knots[i + r] = knots[i];
  }
  // Compute alpha
  array2<double> alpha(degree - s, r + 1, 0.0);
  for (int j = 1; j <= r; ++j) {
    int L = span - degree + j;
    for (int i = 0; i <= degree - j - s; ++i) {
      alpha(i, j)
          = (knot - knots[L + i]) / (knots[i + span + 1] - knots[L + i]);
    }
  }

  // Create a temporary container for affected control points per row/column
  std::vector<VectorNd<Dim>> tmp(degree + 1);

  if (along_u) {
    // Create new control points with additional rows
    new_cp.resize(cp.rows() + r, cp.cols());

    // Update control points
    // Each row is a u-isocurve, each col is a v-isocurve
    for (int col = 0; col < cp.cols(); ++col) {
      // Copy unaffected control points
      for (int i = 0; i <= span - degree; ++i) {
        new_cp(i, col) = cp(i, col);
      }
      for (int i = span - s; i < cp.rows(); ++i) {
        new_cp(i + r, col) = cp(i, col);
      }
      // Copy affected control points to temp array
      for (int i = 0; i < degree - s + 1; ++i) {
        tmp[i] = cp(span - degree + i, col);
      }
      // Insert knot
      for (int j = 1; j <= r; ++j) {
        int L = span - degree + j;
        for (int i = 0; i <= degree - j - s; ++i) {
          double a = alpha(i, j);
          tmp[i]   = (1 - a) * tmp[i] + a * tmp[i + 1];
        }
        new_cp(L, col)                = tmp[0];
        new_cp(span + r - j - s, col) = tmp[degree - j - s];
      }
      int L = span - degree + r;
      for (int i = L + 1; i < span - s; ++i) {
        new_cp(i, col) = tmp[i - L];
      }
    }
  }
  else {
    // Create new control points with additional columns
    new_cp.resize(cp.rows(), cp.cols() + r);

    // Update control points
    // Each row is a u-isocurve, each col is a v-isocurve
    for (int row = 0; row < cp.rows(); ++row) {
      // Copy unaffected control points
      for (int i = 0; i <= span - degree; ++i) {
        new_cp(row, i) = cp(row, i);
      }
      for (int i = span - s; i < cp.cols(); ++i) {
        new_cp(row, i + r) = cp(row, i);
      }
      // Copy affected control points to temp array
      for (int i = 0; i < degree - s + 1; ++i) {
        tmp[i] = cp(row, span - degree + i);
      }
      // Insert knot
      for (int j = 1; j <= r; ++j) {
        int L = span - degree + j;
        for (int i = 0; i <= degree - j - s; ++i) {
          double a = alpha(i, j);
          tmp[i]   = (1 - a) * tmp[i] + a * tmp[i + 1];
        }
        new_cp(row, L)                = tmp[0];
        new_cp(row, span + r - j - s) = tmp[degree - j - s];
      }
      int L = span - degree + r;
      for (int i = L + 1; i < span - s; ++i) {
        new_cp(row, i) = tmp[i - L];
      }
    }
  }
}

/**
 * Split the surface into two along given parameter direction
 * @param[in] degree Degree of surface along given direction
 * @param[in] knots Knot vector of surface along given direction
 * @param[in] control_points Array of control points
 * @param[in] param Parameter to split curve
 * @param[in] along_u Whether the direction to split along is the
 * u-direction
 * @param[out] left_knots Knots of the left part of the curve
 * @param[out] left_control_points Control points of the left part of the
 * curve
 * @param[out] right_knots Knots of the right part of the curve
 * @param[out] right_control_points Control points of the right part of the
 * curve
 */
template <int Dim>
void surfaceSplit(
    int degree,
    const std::vector<double>& knots,
    const array2<VectorNd<Dim>>& control_points,
    double param,
    bool along_u,
    std::vector<double>& left_knots,
    array2<VectorNd<Dim>>& left_control_points,
    std::vector<double>& right_knots,
    array2<VectorNd<Dim>>& right_control_points) {
  std::vector<double> tmp_knots;
  array2<VectorNd<Dim>> tmp_cp;

  int span = find_span(degree, knots, param);
  int r    = degree - knot_multiplicity(knots, span);
  internal::surfaceKnotInsert(
      degree, knots, control_points, param, r, along_u, tmp_knots, tmp_cp);

  left_knots.clear();
  right_knots.clear();

  int span_l = find_span(degree, tmp_knots, param) + 1;
  for (int i = 0; i < span_l; ++i) {
    left_knots.push_back(tmp_knots[i]);
  }
  left_knots.push_back(param);

  for (int i = 0; i < degree + 1; ++i) {
    right_knots.push_back(param);
  }
  for (std::size_t i = span_l; i < tmp_knots.size(); ++i) {
    right_knots.push_back(tmp_knots[i]);
  }

  int ks = span - degree + 1;
  if (along_u) {
    size_t ii = 0;
    left_control_points.resize(ks + r, tmp_cp.cols());
    for (int i = 0; i < ks + r; ++i) {
      for (int j = 0; j < tmp_cp.cols(); ++j) {
        left_control_points[ii++] = tmp_cp(i, j);
      }
    }
    ii = 0;
    right_control_points.resize(tmp_cp.rows() - ks - r + 1, tmp_cp.cols());
    for (int i = ks + r - 1; i < tmp_cp.rows(); ++i) {
      for (int j = 0; j < tmp_cp.cols(); ++j) {
        right_control_points[ii++] = tmp_cp(i, j);
      }
    }
  }
  else {
    size_t ii = 0;
    left_control_points.resize(tmp_cp.rows(), ks + r);
    for (int i = 0; i < tmp_cp.rows(); ++i) {
      for (int j = 0; j < ks + r; ++j) {
        left_control_points[ii++] = tmp_cp(i, j);
      }
    }
    ii = 0;
    right_control_points.resize(tmp_cp.rows(), tmp_cp.cols() - ks - r + 1);
    for (int i = 0; i < tmp_cp.rows(); ++i) {
      for (int j = ks + r - 1; j < tmp_cp.cols(); ++j) {
        right_control_points[ii++] = tmp_cp(i, j);
      }
    }
  }
}

/**
 * Convert a 2D list of nd points in homogenous coordinates to cartesian
 * coordinates by perspective division
 * @param[in] ptsws Points in homogenous coordinates
 * @param[out] pts Points in cartesian coordinates
 * @param[out] ws Homogenous weights
 */
template <int Dim>
inline void homo2cart(
    const array2<VectorNd<Dim>>& ptsws,
    array2<VectorNd<Dim - 1>>& pts,
    array2<double>& ws) {
  pts.resize(ptsws.rows(), ptsws.cols());
  ws.resize(ptsws.rows(), ptsws.cols());
  for (int i = 0; i < ptsws.rows(); ++i) {
    for (int j = 0; j < ptsws.cols(); ++j) {
      const VectorNd<Dim>& ptw_ij = ptsws(i, j);
      double w_ij                 = ptw_ij[Dim - 1];
      pts(i, j)                   = ptw_ij.homo2cart();
      ws(i, j)                    = w_ij;
    }
  }
}

/**
 * Convert 2D list of points in cartesian coordinates to homogenous
 * coordinates
 * @param[in] pts Points in cartesian coordinates
 * @param[in] ws Weights
 * @return Points in homogenous coordinates
 */
template <int Dim>
inline array2<VectorNd<Dim + 1>> cart2homo(
    const array2<VectorNd<Dim>>& pts, const array2<double>& ws) {
  array2<VectorNd<Dim + 1>> Cw(pts.rows(), pts.cols());
  for (int i = 0; i < pts.rows(); ++i) {
    for (int j = 0; j < pts.cols(); ++j) {
      Cw(i, j) = pts(i, j).cart2homo(ws(i, j));
    }
  }
  return Cw;
}

} // namespace crobot::math::internal

