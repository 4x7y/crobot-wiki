#pragma once

#include <memory>

#include "crobot/ik/variable.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::ik {

class ObjectPoseVariables : public ik::Variable {
public:
  using Ptr = std::shared_ptr<ObjectPoseVariables>;

  ObjectPoseVariables(const std::string& name);

  void set_values_all(const math::VectorXd& x) override;

  math::VectorXd get_values_all() const override;

  // Each variable has an upper and lower bound set here
  optimization::VecBound get_bounds_all() const override;

private:
  // xpos(3) + xquat(4)
  math::VectorXd m_pose;
  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

