#pragma once

#include "crobot/ik/cost.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"

namespace crobot::ik {

class BodyPosePotentialCost : public ik::Cost {
public:
  using Ptr      = std::shared_ptr<BodyPosePotentialCost>;
  using ConstPtr = std::shared_ptr<const BodyPosePotentialCost>;

  BodyPosePotentialCost(
      const std::string& name, const math::VectorXd& pose_est);

  // Documentation inherited
  double get_cost(const math::VectorXd& x) const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

  void set_body_xpos_est(
      const math::Vector3d& xpos, const math::Quaternion& xquat);

  void set_body_xpos_est(const math::VectorXd& pose);

private:
  math::Vector3d m_xpos_est;
  math::Vector4d m_xquat_est;
  double m_weight_f1;
  double m_weight_f2;
};

} // namespace crobot::ik

