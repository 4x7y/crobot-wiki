#pragma once

#include "crobot/ik/variable.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/optimization/composite.hpp"

namespace crobot::ik {

class QposVariables : public ik::Variable {
public:
  QposVariables(
      const kdl::ConstRigidBodyTreePtr& rbtree,
      bool limited,
      const std::string& name);

  // Documentation inherited
  void set_values_all(const math::VectorXd& x);

  // Documentation inherited
  math::VectorXd get_values_all() const override;

  // Documentation inherited
  optimization::VecBound get_bounds_all() const override;

  /**
   * @brief Fix a joint to the given value.
   *
   * @param joint_name Name of the joint to fix
   * @param qpos Value to fix the joint to
   */
  void fix_joint(const std::string& joint_name, const math::VectorXd& qpos);

  /**
   * @brief Fix joints in the given range to the given value.
   * @param range Range of joints to fix, start index and number of joints
   * @param qpos Value to fix the joints to
   * @note This function is useful for fixing the joints of a robot arm to
   *      a given configuration.
   * @note In shadow + wam configeration, the first 24 joints are fixed
   */
  void fix_joint_range(
      const std::pair<int, int>& range, const math::VectorXd& qpos);

private:
  math::VectorXd m_qpos;
  math::VectorXd m_vars;
  kdl::ConstRigidBodyTreePtr m_rbtree;
  bool m_is_limited;
};

} // namespace crobot::ik

