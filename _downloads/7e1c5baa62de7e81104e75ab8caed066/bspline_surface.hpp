#pragma once

#include <tuple>
#include <vector>

#include "crobot/math/array.hpp"
#include "crobot/math/bspline.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/spline.hpp"
#include "crobot/math/spline_surface.hpp"

namespace crobot::math {

// Forward declaration
struct RationalSurface;

/**
 * @brief Struct for representing a B-Spline (non-rational NURBS) surface.
 */
struct BSplineSurface : public SplineSurface
{
  std::size_t degree_u, degree_v;
  std::vector<double> knots_u, knots_v;
  array2<Vector3d> control_points;

  BSplineSurface() = default;

  explicit BSplineSurface(const RationalSurface& srf);

  BSplineSurface(
      std::size_t degree_u,
      std::size_t degree_v,
      const std::vector<double>& knots_u,
      const std::vector<double>& knots_v,
      array2<Vector3d> control_points);

  BSplineSurface(const std::string& filename);

  static std::shared_ptr<const BSplineSurface> from_yaml_node(
      const YAML::Node& node);

  void scale(Vector3d scale);

  void translate(Vector3d vec);

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

  /**
   * @brief Insert knots in the surface along u-direction
   *
   * @param[in] u Knot value to insert
   * @param[in] repeat Number of times to insert
   * @return New BSplineSurface object after knot insertion
   */
  BSplineSurface insert_knot_along_u(double u, int repeat = 1) const;

  /**
   * @brief Insert knots in the surface along v-direction
   *
   * @param[in] v Knot value to insert
   * @param[in] repeat Number of times to insert
   * @return New BSplineSurface object after knot insertion
   */
  BSplineSurface insert_knot_along_v(double v, int repeat = 1) const;

  /**
   * @brief Split a surface into two along u-direction
   *
   * @param[in] u Parameter along u-direction to split the surface
   * @return Tuple with first and second half of the surfaces
   */
  std::tuple<BSplineSurface, BSplineSurface> split_along_u(double u) const;

  /**
   * @brief Split a surface into two along v-direction
   *
   * @param[in] v Parameter along v-direction to split the surface
   * @return Tuple with first and second half of the surfaces
   */
  std::tuple<BSplineSurface, BSplineSurface> split_along_v(double v) const;

  YAML::Node to_yaml_node() const override;

private:
  std::string m_filename;
};

using BSplineSurfacePtr      = std::shared_ptr<BSplineSurface>;
using ConstBSplineSurfacePtr = std::shared_ptr<const BSplineSurface>;

} // namespace crobot::math

#include "crobot/math/detail/bspline_surface_impl.hpp"

