#pragma once

#include <vector>

#include "crobot/math/array.hpp"
#include "crobot/math/bspline_surface.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::math {

/**
 * @brief Struct for representing a NURBS (Non-Uniform Rational B-Spline)
 * surface.
 */
struct RationalSurface : public SplineSurface
{
  std::size_t degree_u, degree_v;
  std::vector<double> knots_u, knots_v;
  array2<Vector3d> control_points;
  array2<double> weights;

  RationalSurface() = default;

  RationalSurface(const BSplineSurface& srf);

  RationalSurface(const BSplineSurface& srf, const array2<double>& weights);

  RationalSurface(
      std::size_t degree_u,
      std::size_t degree_v,
      const std::vector<double>& knots_u,
      const std::vector<double>& knots_v,
      const array2<Vector3d>& control_points,
      const array2<double>& weights);

  // Documentation inherited
  Vector3d evaluate(double u, double v) const override;

  // Documentation inherited
  array2<Vector3d> derivatives(int num_ders, double u, double v) const override;

  // Documentation inherited
  std::tuple<Vector3d, Vector3d> tangent(double u, double v) const override;

  // Documentation inherited
  Vector3d normal(double u, double v) const override;

  // Documentation inherited
  bool is_valid() const override;

  // Documentation inherited
  bool is_closed_along_u() const override;

  // Documentation inherited
  bool is_closed_along_v() const override;

  YAML::Node to_yaml_node() const override;

  /**
   * @brief Insert knots in the rational surface along u-direction
   *
   * @param[in] u Knot value to insert
   * @param[in] repeat Number of times to insert
   * @return New RationalSurface object after knot insertion
   */
  RationalSurface insert_knot_along_u(double u, int repeat = 1) const;

  /**
   * Insert knots in the rational surface along v-direction
   *
   * @param[in] v Knot value to insert
   * @param[in] repeat Number of times to insert
   * @return New RationalSurface object after knot insertion
   */
  RationalSurface insert_knot_along_v(double v, int repeat = 1) const;

  /**
   * Split a rational surface into two along u-direction
   *
   * @param[in] u Parameter along u-direction to split the surface
   * @return Tuple with first and second half of the surfaces
   */
  std::tuple<RationalSurface, RationalSurface> split_along_u(double u) const;

  /**
   * @brief Split a rational surface into two along v-direction
   *
   * @param[in] v Parameter along v-direction to split the surface
   * @return Tuple with first and second half of the surfaces
   */
  std::tuple<RationalSurface, RationalSurface> split_along_v(double v) const;
};

} // namespace crobot::math

