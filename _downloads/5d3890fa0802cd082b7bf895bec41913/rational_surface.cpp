#include "crobot/math/rational_surface.hpp"

#include "crobot/math/bspline_surface.hpp"
#include "crobot/math/spline.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
RationalSurface::RationalSurface(
    const BSplineSurface& srf, const array2<double>& weights)
  : degree_u(srf.degree_u),
    degree_v(srf.degree_v),
    knots_u(srf.knots_u),
    knots_v(srf.knots_v),
    control_points(srf.control_points),
    weights(weights) {
}

//==============================================================================
RationalSurface::RationalSurface(const BSplineSurface& srf)
  : RationalSurface(
      srf,
      array2<double>(
          srf.control_points.rows(), srf.control_points.cols(), 1.0)) {
}

//==============================================================================
RationalSurface::RationalSurface(
    std::size_t degree_u,
    std::size_t degree_v,
    const std::vector<double>& knots_u,
    const std::vector<double>& knots_v,
    const array2<Vector3d>& control_points,
    const array2<double>& weights)
  : degree_u(degree_u),
    degree_v(degree_v),
    knots_u(knots_u),
    knots_v(knots_v),
    control_points(control_points),
    weights(weights) {
}

//==============================================================================
Vector3d RationalSurface::evaluate(double u, double v) const {
  // Compute homogenous coordinates of control points
  array2<Vector4d> Cw;
  Cw.resize(control_points.rows(), control_points.cols());
  for (int i = 0; i < control_points.rows(); i++) {
    for (int j = 0; j < control_points.cols(); j++) {
      Cw(i, j) = control_points(i, j).cart2homo(weights(i, j));
    }
  }

  // Compute point using homogenous coordinates
  Vector4d pointw
      = internal::evaluate(degree_u, degree_v, knots_u, knots_v, Cw, u, v);

  // Convert back to cartesian coordinates
  return pointw.homo2cart();
}

//==============================================================================
array2<Vector3d> RationalSurface::derivatives(
    int num_ders, double u, double v) const {

  array2<Vector4d> homo_cp;
  homo_cp.resize(control_points.rows(), control_points.cols());
  for (int i = 0; i < control_points.rows(); ++i) {
    for (int j = 0; j < control_points.cols(); ++j) {
      homo_cp(i, j) = control_points(i, j).cart2homo(weights(i, j));
    }
  }

  array2<Vector4d> homo_ders = internal::derivatives(
      degree_u, degree_v, knots_u, knots_v, homo_cp, num_ders, u, v);

  array2<Vector3d> Aders;
  Aders.resize(num_ders + 1, num_ders + 1);
  for (int i = 0; i < homo_ders.rows(); ++i) {
    for (int j = 0; j < homo_ders.cols(); ++j) {
      Aders(i, j) = homo_ders(i, j).homo2cart_truncate();
    }
  }

  array2<Vector3d> surf_ders(num_ders + 1, num_ders + 1);
  for (int k = 0; k < num_ders + 1; ++k) {
    for (int l = 0; l < num_ders - k + 1; ++l) {
      auto der = Aders(k, l);

      for (int j = 1; j < l + 1; ++j) {
        der -= internal::binomial(l, j) * homo_ders(0, j).w()
               * surf_ders(k, l - j);
      }

      for (int i = 1; i < k + 1; ++i) {
        der -= internal::binomial(k, i) * homo_ders(i, 0).w()
               * surf_ders(k - i, l);

        Vector3d tmp = Vector3d::Zero();
        for (int j = 1; j < l + 1; ++j) {
          tmp -= internal::binomial(l, j) * homo_ders(i, j).w()
                 * surf_ders(k - 1, l - j);
        }

        der -= internal::binomial(k, i) * tmp;
      }

      der *= 1 / homo_ders(0, 0).w();
      surf_ders(k, l) = der;
    }
  }
  return surf_ders;
}

//==============================================================================
std::tuple<Vector3d, Vector3d> RationalSurface::tangent(
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
Vector3d RationalSurface::normal(double u, double v) const {
  array2<Vector3d> ptder = derivatives(1, u, v);
  Vector3d du            = ptder(1, 0);
  Vector3d dv            = ptder(0, 1);
  Vector3d n             = dv.cross(du);
  double n_len           = n.norm();
  if (!is_zero(n_len)) {
    n /= n_len;
  }
  return n;
}

//==============================================================================
bool RationalSurface::is_valid() const {
  return internal::is_surface_valid(
      degree_u, degree_v, knots_u, knots_v, control_points, weights);
}

//==============================================================================
bool RationalSurface::is_closed_along_u() const {
  return internal::is_2d_array_closed_u(degree_u, control_points)
         && internal::is_knot_vector_closed(degree_u, knots_u)
         && internal::is_2d_array_closed_u(degree_u, weights);
}

//==============================================================================
bool RationalSurface::is_closed_along_v() const {
  return internal::is_2d_array_closed_v(degree_v, control_points)
         && internal::is_knot_vector_closed(degree_v, knots_v)
         && internal::is_2d_array_closed_v(degree_v, weights);
}

//==============================================================================
RationalSurface RationalSurface::insert_knot_along_u(
    double u, int repeat) const {
  RationalSurface new_srf;
  new_srf.degree_u = degree_u;
  new_srf.degree_v = degree_v;
  new_srf.knots_v  = knots_v;

  // Original control points in homogenous coordinates
  array2<Vector4d> Cw(control_points.rows(), control_points.cols());
  for (int i = 0; i < control_points.rows(); ++i) {
    for (int j = 0; j < control_points.cols(); ++j) {
      Cw(i, j) = control_points(i, j).cart2homo(weights(i, j));
    }
  }

  // New knots and new homogenous control points after knot insertion
  std::vector<double> new_knots_u;
  array2<Vector4d> new_Cw;
  internal::surfaceKnotInsert(
      degree_u, knots_u, Cw, u, repeat, true, new_srf.knots_u, new_Cw);

  // Convert back to cartesian coordinates
  new_srf.control_points.resize(new_Cw.rows(), new_Cw.cols());
  new_srf.weights.resize(new_Cw.rows(), new_Cw.cols());
  for (int i = 0; i < new_Cw.rows(); ++i) {
    for (int j = 0; j < new_Cw.cols(); ++j) {
      new_srf.control_points(i, j) = new_Cw(i, j).homo2cart();
      new_srf.weights(i, j)        = new_Cw(i, j).w();
    }
  }
  return new_srf;
}

//==============================================================================
RationalSurface RationalSurface::insert_knot_along_v(
    double v, int repeat) const {
  RationalSurface new_srf;
  new_srf.degree_u = degree_u;
  new_srf.degree_v = degree_v;
  new_srf.knots_u  = knots_u;
  // Original control points in homogenous coordinates
  array2<Vector4d> Cw(control_points.rows(), control_points.cols());
  for (int i = 0; i < control_points.rows(); ++i) {
    for (int j = 0; j < control_points.cols(); ++j) {
      Cw(i, j) = control_points(i, j).cart2homo(weights(i, j));
    }
  }

  // New knots and new homogenous control points after knot insertion
  std::vector<double> new_knots_v;
  array2<Vector4d> new_Cw;
  internal::surfaceKnotInsert(
      degree_v, knots_v, Cw, v, repeat, false, new_srf.knots_v, new_Cw);

  // Convert back to cartesian coordinates
  new_srf.control_points.resize(new_Cw.rows(), new_Cw.cols());
  new_srf.weights.resize(new_Cw.rows(), new_Cw.cols());
  for (int i = 0; i < new_Cw.rows(); ++i) {
    for (int j = 0; j < new_Cw.cols(); ++j) {
      new_srf.control_points(i, j) = new_Cw(i, j).homo2cart();
      new_srf.weights(i, j)        = new_Cw(i, j).w();
    }
  }
  return new_srf;
}

//==============================================================================
std::tuple<RationalSurface, RationalSurface> RationalSurface::split_along_u(
    double u) const {
  RationalSurface left, right;
  left.degree_u  = degree_u;
  left.degree_v  = degree_v;
  left.knots_v   = knots_v;
  right.degree_u = degree_u;
  right.degree_v = degree_v;
  right.knots_v  = knots_v;

  // Compute homogenous coordinates of control points and weights
  array2<Vector4d> Cw = internal::cart2homo(control_points, weights);

  // Split surface with homogenous coordinates
  array2<Vector4d> left_Cw, right_Cw;
  internal::surfaceSplit(
      degree_u,
      knots_u,
      Cw,
      u,
      true,
      left.knots_u,
      left_Cw,
      right.knots_u,
      right_Cw);

  // Convert back to cartesian coordinates
  internal::homo2cart(left_Cw, left.control_points, left.weights);
  internal::homo2cart(right_Cw, right.control_points, right.weights);

  return std::make_tuple(std::move(left), std::move(right));
}

//==============================================================================
std::tuple<RationalSurface, RationalSurface> RationalSurface::split_along_v(
    double v) const {
  RationalSurface left, right;
  left.degree_u  = degree_u;
  left.degree_v  = degree_v;
  left.knots_u   = knots_u;
  right.degree_u = degree_u;
  right.degree_v = degree_v;
  right.knots_u  = knots_u;

  // Compute homogenous coordinates of control points and weights
  array2<Vector4d> Cw = internal::cart2homo(control_points, weights);

  // Split surface with homogenous coordinates
  array2<Vector4d> left_Cw, right_Cw;
  internal::surfaceSplit(
      degree_v,
      knots_v,
      Cw,
      v,
      false,
      left.knots_v,
      left_Cw,
      right.knots_v,
      right_Cw);

  // Convert back to cartesian coordinates
  internal::homo2cart(left_Cw, left.control_points, left.weights);
  internal::homo2cart(right_Cw, right.control_points, right.weights);

  return std::make_tuple(std::move(left), std::move(right));
}

//==============================================================================
YAML::Node RationalSurface::to_yaml_node() const {
  YAML::Node node;
  node["type"]     = "RationalSurface";
  node["degree_u"] = degree_u;
  node["degree_v"] = degree_v;
  // not DONE!
  return node;
}
