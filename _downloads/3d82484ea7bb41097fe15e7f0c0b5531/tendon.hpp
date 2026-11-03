#pragma once

#include <memory>
#include <string>

#include "crobot/kdl/rigid_body_joint.hpp"
#include "crobot/kdl/transmission.hpp"

namespace crobot::kdl {

class Tendon;
using TendonPtr      = std::shared_ptr<Tendon>;
using ConstTendonPtr = std::shared_ptr<const Tendon>;
class RigidBodyTree;
using ConstRigidBodyTreePtr = std::shared_ptr<const RigidBodyTree>;

class Tendon : public Transmission {
public:
  /**
   * @brief Construct a tendon whose length is defined as a linear combination
   * of joint positions.
   */
  Tendon(ConstRigidBodyTreePtr rbtree, const std::string& name);

  /**
   * @brief Destroy the Tendon object.
   */
  ~Tendon() = default;

  /**
   * @brief Add a joint to the tendon.
   */
  void add_joint(ConstJointPtr joint, double coeff);

  /**
   * @brief Get the joint configuration indices in the RigidBodyTree
   * configuration vector @c qpos .
   */
  std::vector<std::pair<int, int>> get_qpos_addr() const override;

  /**
   * @brief Get the joint velocity (acceleration) indices in the RigidBodyTree
   * velocity (acceleration) vector @c qvel ( @c qacc ).
   */
  std::vector<std::pair<int, int>> get_qvel_addr() const override;

  /**
   * @brief Get the transmission coefficients for each joint.
   */
  std::vector<double> get_transmission_coeffs() const override;

protected:
  // The rigidBodyTree that the tendon associated with.
  ConstRigidBodyTreePtr m_rbtree;

  // Name of the tendon.
  std::string m_name;

  // Joint added to the fixed tendon. Only scalar joints (revolute and
  // prismatic) can be referenced here.
  std::vector<ConstJointPtr> m_joints;

  // Scalar coefficient multiplying the position or angle of the specified
  // joint.
  std::vector<double> m_coeffs;

  friend RigidBodyTree;
};

} // namespace crobot::kdl

