#pragma once

#include <memory>

#include "crobot/ik/constraint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class PointContactConstraint : public ik::Constraint {
public:
  using Ptr = std::shared_ptr<PointContactConstraint>;

  PointContactConstraint(
      const std::string& name,
      const math::Vector3d& bpos,
      const math::Vector3d& xpos,
      double tolerance);

  // Documentation inherited
  math::VectorXd get_values(const math::VectorXd& x) const override;

  // Documentation inherited
  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

  void set_xpos(const math::Vector3d& xpos);

private:
  math::Vector3d m_bpos;
  math::Vector3d m_xpos_target;

  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

