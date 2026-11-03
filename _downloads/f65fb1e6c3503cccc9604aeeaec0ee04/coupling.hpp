#pragma once

#include "crobot/kdl/rigid_body_joint.hpp"

namespace crobot::kdl {

class RigidBodyTree;

/**
 * @brief This class constrains the position or angle of one joint (
 * @param joint1 ) to be a polynomial of another joint ( @param joint2 ). Only
 * scalar joint types (slide and hinge) can be used.
 */
class Coupling {
public:
  /**
   * @brief Construct a new Coupling object
   */
  Coupling(
      const std::string& name,
      ConstJointPtr joint1,
      ConstJointPtr joint2,
      const math::VectorXd& poly);

  /**
   * @brief Get the coefficients that couple the two joint velocities together.
   *
   * @param qpos2 Joint configuration of the second joint.
   */
  std::pair<double, double> get_qvel_coeffs(double qpos2) const;

protected:
  // Name of the couping constraint.
  std::string m_name;

  // The first joint.
  ConstJointPtr m_joint1;

  // The second joint.
  ConstJointPtr m_joint2;

  // Coefficients a0 ... a4 of the polynomial. If the two joint values are
  // y and x, and their reference positions (corresponding to the joint values
  // in the initial model configuration) are y0 and x0, the constraint is: y-y0
  // = a0 + a1*(x-x0) + a2*(x-x0)^2 + a3*(x-x0)^3 + a4*(x-x0)^4 Omitting the
  // second joint is equivalent to setting x = x0, in which case the constraint
  // is y = y0 + a0.
  math::VectorXd m_poly;

  // Derivative of the polynomial m_poly.
  math::VectorXd m_polyder;

  friend class RigidBodyTree;
};

using CouplingPtr      = std::shared_ptr<Coupling>;
using ConstCouplingPtr = std::shared_ptr<const Coupling>;

} // namespace crobot::kdl

