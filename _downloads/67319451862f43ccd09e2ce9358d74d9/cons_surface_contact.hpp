#pragma once

#include <memory>

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class SurfaceContactConstraint : public ik::Constraint {
public:
  SurfaceContactConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body1,
      std::shared_ptr<const math::SurfacePatch> patch1,
      const std::string& varname_patch1,
      const std::string& body2,
      std::shared_ptr<const math::SurfacePatch> patch2,
      const std::string& varname_patch2,
      double tolerance);

  // Documentation inherited
  math::VectorXd get_values(const math::VectorXd& x) const override;

  // Documentation inherited
  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

  /**
   * @brief Get the first surface patch.
   *
   * @return The first surface patch.
   */
  math::SurfacePatch::ConstPtr get_patch1() const;

  /**
   * @brief Get the second surface patch.
   *
   * @return The second surface patch.
   */
  math::SurfacePatch::ConstPtr get_patch2() const;

  /**
   * @brief Get the contact frame on the first patch.
   *
   * @return Transformation matrix that represents the pose of the contact frame
   * on the first surface patch.
   */
  Eigen::Matrix4d get_contact_frame_patch1() const;

  /**
   * @brief Get the contact frame on the second patch.
   *
   * @return Transformation matrix that represents the pose of the contact frame
   * on the second surface patch.
   */
  Eigen::Matrix4d get_contact_frame_patch2() const;

private:
  std::tuple<math::Vector2d, math::Vector2d, math::VectorXd>
  parse_dependent_vars(const math::VectorXd& x) const;

  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body1;
  kdl::ConstRigidBodyPtr m_body2;
  math::SurfacePatch::ConstPtr m_patch1;
  math::SurfacePatch::ConstPtr m_patch2;
  std::string m_varname_patch1;
  std::string m_varname_patch2;

  std::vector<int> m_qpos_addr;
  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

