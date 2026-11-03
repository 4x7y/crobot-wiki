#pragma once

#include "crobot/math/matrix.hpp"

#include <memory>

namespace crobot::ik {

class JacobianIKTarget {
public:
  using Ptr      = std::shared_ptr<JacobianIKTarget>;
  using ConstPtr = std::shared_ptr<const JacobianIKTarget>;

  virtual ~JacobianIKTarget() = default;

  /**
   * @brief
   *
   * @param tforms
   * @return std::pair<math::VectorXd, math::MatrixXd>
   */
  virtual std::pair<math::VectorXd, math::MatrixXd> compute_value_and_jacobian(
      const std::vector<math::Matrix4d>& tforms) const
      = 0;

  /**
   * @brief Get the ndofs object
   *
   * @return int
   */
  virtual int get_ndofs() const = 0;

  /**
   * @brief Get the qvel addrs object
   *
   * @return std::vector<int>
   */
  virtual std::vector<int> get_qvel_addrs() const = 0;
};

} // namespace crobot::ik

