#pragma once

#include <memory>

#include "crobot/ik/constraint.hpp"

namespace crobot::ik {

class SurfaceParamsConstraint : public ik::Constraint {
public:
  using Ptr = std::shared_ptr<SurfaceParamsConstraint>;

  SurfaceParamsConstraint(
      const std::string& name,
      const std::string& varname,
      const math::Vector2d& uv_est,
      double tolerance);

  // Documentation inherited
  math::VectorXd get_values(const math::VectorXd& x) const override;

  // Documentation inherited
  optimization::VecBound get_bounds() const override;

  // Documentation inherited
  void fill_jacobian_block(
      Variable::ConstPtr varsets,
      math::SparseJacobian& jac_block) const override;

  void set_uv_est(const math::Vector2d& uv);

private:
  std::string m_varname;
  math::Vector2d m_uv_est;

  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

