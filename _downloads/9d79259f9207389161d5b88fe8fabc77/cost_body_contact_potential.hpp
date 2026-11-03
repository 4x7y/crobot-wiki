#pragma once

#include "crobot/ik/cost.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class BodyContactPotentialCost : public ik::Cost {
public:
  using Ptr      = std::shared_ptr<BodyContactPotentialCost>;
  using ConstPtr = std::shared_ptr<const BodyContactPotentialCost>;

  BodyContactPotentialCost(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body_obj,
      std::shared_ptr<const math::SurfacePatch> patch_obj,
      const std::string& varname_patch_obj,
      const std::string& body_rob,
      std::shared_ptr<const math::SurfacePatch> patch_rob,
      const std::string& varname_patch_rob,
      const math::Vector3d& xpos_screw);

  // Documentation inherited
  double get_cost(const math::VectorXd& x) const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

  void set_qpos(const math::VectorXd& qpos);

  void set_screw_xpos(const math::Vector3d& xpos_screw);

private:
  std::tuple<math::Vector2d, math::Vector2d, math::VectorXd, math::VectorXd>
  parse_dependent_vars(const math::VectorXd& x) const;

  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body_obj;
  kdl::ConstRigidBodyPtr m_body_rob;
  math::SurfacePatch::ConstPtr m_patch_obj;
  math::SurfacePatch::ConstPtr m_patch_rob;
  std::string m_varname_patch_obj;
  std::string m_varname_patch_rob;

  math::VectorXd m_qpos;
  std::vector<int> m_pose_addr;

  math::Vector3d m_xpos_screw;
};

} // namespace crobot::ik

