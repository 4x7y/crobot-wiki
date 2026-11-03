#pragma once
#include <exception>
#include <stdexcept>
#include <vector>
#include <memory>

#include "crobot/math/geometry.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::math {

class Curve : public Geometry {
public:
  using Ptr      = std::shared_ptr<Curve>;
  using ConstPtr = std::shared_ptr<const Curve>;

  /**
   * @brief Evaluate point on a spline curve
   *
   * @param[in] u Parameter to evaluate the curve at.
   * @return point Resulting point on the curve.
   */
  virtual Vector3d evaluate(double u) const = 0;

  /**
   * @brief Evaluate derivatives of a spline curve
   *
   * @param[in] num_ders Number of times to differentiate.
   * @param[inout] curve_ders Derivatives of the curve at u.
   *
   * @note curve_ders[n] is the nth derivative at u, where n is between 0 and
   * num_ders-1.
   */
  virtual std::vector<Vector3d> derivatives(int num_ders, double u) const = 0;

  /**
   * @brief Evaluate the tangent of a spline curve
   *
   * @return Unit tangent of the curve at u.
   */
  virtual Vector3d tangent(double u) const;

  /**
   * @brief Evaluate the Jacobian matrix of the unit tangent vector at the given
   * parameter.
   *
   * @param u Curve parameter
   * @return Vector3d
   */
  virtual Vector3d tangent_jacb(double u) const;

  /**
   * @brief Returns whether the curve is valid
   *
   * @return Whether valid
   */
  virtual bool is_valid() const = 0;

  /**
   * @brief Checks whether the rational curve is closed
   *
   * @return Whether closed
   */
  virtual bool is_closed() const = 0;
};

} // namespace crobot::math

