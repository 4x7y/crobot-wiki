#pragma once

#include <vector>

#include "crobot/math/curve.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::math {

struct BSplineCurve;

/**
 * @brief Struct for holding a rational B-spline curve
 */
struct RationalCurve : public Curve
{
  std::size_t degree;
  std::vector<double> knots;
  std::vector<Vector3d> control_points;
  std::vector<double> weights;

  RationalCurve() = default;
  RationalCurve(const BSplineCurve& crv);
  RationalCurve(const BSplineCurve& crv, const std::vector<double>& weights);
  RationalCurve(
      std::size_t degree,
      const std::vector<double>& knots,
      const std::vector<Vector3d>& control_points,
      const std::vector<double> weights);

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
   * @brief Insert knots in the rational curve
   *
   * @param[in] u Parameter to insert knot at
   * @param[in] repeat Number of times to insert
   * @return New RationalCurve object with #repeat knots inserted at u
   */
  RationalCurve insert_knot(double u, int repeat = 1) const;

  /**
   * Split a rational curve into two
   *
   * @param[in] u Parameter to split at
   * @return Tuple with first half and second half of the curve
   */
  std::tuple<RationalCurve, RationalCurve> split(double u) const;
};

} // namespace crobot::math

