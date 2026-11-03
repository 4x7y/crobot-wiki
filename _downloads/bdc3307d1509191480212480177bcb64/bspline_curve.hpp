#pragma once

#include <vector>

#include "crobot/math/bspline.hpp"
#include "crobot/math/curve.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/spline.hpp"

namespace crobot::math {

// Forward declaration
struct RationalCurve;

/**
 * @brief Struct for holding a polynomial B-spline curve
 */
struct BSplineCurve : public Curve
{
  std::size_t degree;
  std::vector<double> knots;
  std::vector<Vector3d> control_points;

  BSplineCurve() = default;
  BSplineCurve(const RationalCurve& crv);
  BSplineCurve(
      std::size_t degree,
      const std::vector<double>& knots,
      const std::vector<Vector3d>& control_points);

  BSplineCurve(const std::string& filename);

  // Documentation inherited
  Vector3d evaluate(double u) const override;

  // Documentation inherited
  std::vector<Vector3d> derivatives(int num_ders, double u) const override;

  // Documentation inherited
  Vector3d tangent(double u) const override;

  // Documentation inherited
  bool is_valid() const override;

  // Documentation inherited
  bool is_closed() const override;

  /**
   * @brief Insert knots in the curve
   *
   * @param[in] u Parameter to insert knot at
   * @param[in] repeat Number of times to insert
   * @return New curve with #repeat knots inserted at u
   */
  BSplineCurve insert_knot(double u, int repeat = 1) const;

  /**
   * @brief Split a curve into two
   *
   * @param[in] u Parameter to split at
   * @return Tuple with first half and second half of the curve
   */
  std::tuple<BSplineCurve, BSplineCurve> split(double u) const;
};

using BSplineCurvePtr      = std::shared_ptr<BSplineCurve>;
using ConstBSplineCurvePtr = std::shared_ptr<const BSplineCurve>;

} // namespace crobot::math

#include "crobot/math/detail/bspline_curve_impl.hpp"

