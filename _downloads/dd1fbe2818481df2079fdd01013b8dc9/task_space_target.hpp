#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::control {

class TaskSpaceTarget {
public:
  virtual ~TaskSpaceTarget() = default;

  virtual std::tuple<math::VectorXd, math::VectorXd, math::MatrixXd> resolve(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const std::vector<math::Matrix4d>& tforms) const = 0;
};

using TaskSpaceTargetPtr      = std::shared_ptr<TaskSpaceTarget>;
using ConstTaskSpaceTargetPtr = std::shared_ptr<const TaskSpaceTarget>;

} // namespace crobot::control

