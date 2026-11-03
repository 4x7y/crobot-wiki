#pragma once

#include <memory>

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class SurfacePointContactConstraint : public ik::Constraint {
public:
  SurfacePointContactConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body1,
      std::shared_ptr<const math::SurfacePatch> patch1,
      const std::string& varname_patch1,
      const math::Vector3d& xpos_body2,
      const math::Vector3d& xvec_body2,
      double tolerance);

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
  kdl::ConstRigidBodyPtr m_body1;
  math::SurfacePatch::ConstPtr m_patch1;
  std::string m_varname_patch1;

  math::Vector3d m_xpos_body2;
  math::Vector3d m_xvec_body2;

  std::vector<int> m_qpos_addr;
  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

