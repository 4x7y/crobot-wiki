#pragma once

#include <memory>

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class ObjectContactConstraint : public ik::Constraint {
public:
  using Ptr = std::shared_ptr<ObjectContactConstraint>;

  ObjectContactConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_obj,
      const math::Vector3d& bpos_obj,
      const math::Vector3d& bvec_obj,
      const std::string& body_rob,
      std::shared_ptr<const math::SurfacePatch> patch_rob,
      const std::string& varname_patch_rob,
      double tolerance_p,
      double tolerance_n);

  // Documentation inherited
  math::VectorXd get_values(const math::VectorXd& x) const override;

  // Documentation inherited
  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

private:
  std::tuple<math::Vector2d, math::VectorXd> parse_dependent_vars(
      const math::VectorXd& x) const;

  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body_obj;
  math::Vector3d m_bpos_obj;
  math::Vector3d m_bvec_obj;

  kdl::ConstRigidBodyPtr m_body_rob;
  math::SurfacePatch::ConstPtr m_patch_rob;
  std::string m_varname_patch_rob;

  std::vector<int> m_pose_addr;

  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

