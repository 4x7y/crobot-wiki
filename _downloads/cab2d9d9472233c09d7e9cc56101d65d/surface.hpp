#pragma once

#include <vector>

#include <yaml-cpp/yaml.h>

#include "crobot/math/array.hpp"
#include "crobot/math/geometry.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::math {

class Surface : public Geometry {
public:
  using Ptr      = std::shared_ptr<Surface>;
  using ConstPtr = std::shared_ptr<const Surface>;

  virtual ~Surface() = default;

  /**
   * @brief Evaluate point on a spline surface
   *
   * @param[in] u Parameter to evaluate the surface at.
   * @param[in] v Parameter to evaluate the surface at.
   * @return Resulting point on the surface at (u, v).
   */
  virtual Vector3d evaluate(double u, double v) const = 0;

  /**
   * @brief Evaluate derivatives on a spline surface
   *
   * @param[in] num_ders Number of times to differentiate
   * @param[in] u Parameter to evaluate the surface at.
   * @param[in] v Parameter to evaluate the surface at.
   * @return surf_ders Derivatives of the surface at (u, v).
   */
  virtual array2<Vector3d> derivatives(
      int num_ders, double u, double v) const = 0;

  /**
   * @brief Evaluate the two orthogonal tangents of a non-rational surface at
   * the given parameters
   *
   * @param u Parameter in the u-direction
   * @param v Parameter in the v-direction
   * @return Tuple with unit tangents along u- and v-directions
   */
  virtual std::tuple<Vector3d, Vector3d> tangent(double u, double v) const;

  /**
   * @brief Evaluate the normal a non-rational surface at the given parameters
   *
   * @param u Parameter in the u-direction
   * @param v Parameter in the v-direction
   * @return Unit normal at of the surface at (u, v)
   */
  virtual Vector3d normal(double u, double v) const;

  /**
   * @brief Evaluate the Jacobian matrix of the normal at the given parameters
   *
   * @param u Parameter in the u-direction
   * @param v Parameter in the v-direction
   * @return Jacbomian matrix of the normal at (u, v)
   */
  virtual Matrix32 normal_jacb(double u, double v) const;

  /**
   * @brief Evaluate the Hessians of the normal at the given parameters
   *
   * @param u Parameter in the u-direction
   * @param v Parameter in the v-direction
   * @return [H_uu, H_uv; H_vu, H_vv] 6x2
   */
  virtual Matrix6X normal_hess(double u, double v) const;

  virtual YAML::Node to_yaml_node() const = 0;
};

} // namespace crobot::math

