#pragma once

#include <memory>

#include "crobot/ik/variable.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class SurfacePatchVariables : public ik::Variable {
public:
  using Ptr = std::shared_ptr<SurfacePatchVariables>;

  SurfacePatchVariables(
      const std::string& name,
      const std::shared_ptr<const math::SurfacePatch>& patch);

  void set_values_all(const math::VectorXd& x) override;

  math::VectorXd get_values_all() const override;

  // Each variable has an upper and lower bound set here
  optimization::VecBound get_bounds_all() const override;

private:
  math::Vector2d m_uv;

  std::shared_ptr<const math::SurfacePatch> m_patch;

  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

