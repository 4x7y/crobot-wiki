#pragma once

#include <memory>

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/curve_segment.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class SurfaceCurveContactConstraint : public ik::Constraint {
public:
  SurfaceCurveContactConstraint(
      const std::string& name,
      kdl::ConstRigidBodyTreePtr rbtree,
      const std::string& body1,
      std::shared_ptr<const math::SurfacePatch> patch,
      const std::string& varname_patch,
      const std::string& body2,
      std::shared_ptr<const math::CurveSegment> segment,
      const std::string& varname_segment,
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
  std::tuple<math::Vector2d, double, math::VectorXd> parse_dependent_vars(
      const math::VectorXd& x) const;

  kdl::ConstRigidBodyTreePtr m_rbtree;
  kdl::ConstRigidBodyPtr m_body1;
  kdl::ConstRigidBodyPtr m_body2;
  std::shared_ptr<const math::SurfacePatch> m_patch;
  std::shared_ptr<const math::CurveSegment> m_segment;
  std::string m_varname_patch;
  std::string m_varname_segment;
  std::vector<int> m_qpos_addr;
  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

