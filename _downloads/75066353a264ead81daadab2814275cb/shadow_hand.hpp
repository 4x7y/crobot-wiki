#pragma once

#include <string>
#include <unordered_map>

#include "crobot/common/vector_mapper.hpp"
#include "crobot/hardware/shadow_config.hpp"
#include "crobot/hardware/shadow_finger.hpp"
#include "crobot/hardware/shadow_joint.hpp"
#include "crobot/hardware/shadow_motor.hpp"
#include "crobot/hardware/shadow_tension_friction.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/simulation/simulation.hpp"

namespace crobot {
namespace hardware {

class ShadowHand {
public:
  using Ptr = std::shared_ptr<ShadowHand>;

  /**
   * @brief Make a default shadow hand model.
   *
   * @return ShadowHand::Ptr The shadow hand model
   */
  static ShadowHand::Ptr MakeShared();

  /**
   * @brief Make a shadow hand model from the given RigidBodyTree.
   *
   * @param model The RigidBodyTree model
   * @return ShadowHand::Ptr The shadow hand model
   */
  static ShadowHand::Ptr MakeShared(kdl::ConstRigidBodyTreePtr model);

  /**
   * @brief Get the internal RigidBodyTree model.
   *
   * @return kdl::ConstRigidBodyTreePtr The internal RigidBodyTree model
   */
  kdl::ConstRigidBodyTreePtr get_model() const;

  /**
   * @brief Get a finger handler by the finger name.
   *
   * @param finger_name Name of the finger
   * @return std::shared_ptr<const ShadowFinger> The finger handler
   */
  ShadowFinger::ConstPtr get_finger(const std::string& finger_name) const;

  /**
   * @brief Get a joint handler by the joint name.
   *
   * @param joint_name Name of the joint
   * @return ShadowJoint::ConstPtr The joint handler
   */
  // ShadowJoint::ConstPtr get_joint(const std::string& joint_name) const;

  /**
   * @brief Get a motor handler by the motor name.
   *
   * @param motor_name Name of the motor
   * @return ShadowJoint::ConstPtr The motor handler
   */
  // ShadowMotor::ConstPtr get_motor(const std::string& motor_name) const;

  /**
   * @brief Get the joint names in the CROBOT order.
   *
   * @return std::vector<std::string> The joint names
   */
  std::vector<std::string> get_joint_names() const;

  /**
   * @brief Print the hand configuration to the console.
   */
  void print_details() const;

  std::vector<double> get_joint_upper_limits_ros() const;
  std::vector<double> get_joint_lower_limits_ros() const;

private:
  /**
   * @brief Construct a new Shadow Hand object with the given hand model.
   *
   * @param model The hand model
   */
  ShadowHand(kdl::ConstRigidBodyTreePtr model);

  kdl::ConstRigidBodyTreePtr m_hand;

  std::unordered_map<std::string, ShadowFinger::Ptr> m_fingers;

  std::vector<TensionFriction::Ptr> m_tension_frictions;

  static std::vector<std::string> ROS_JOINT_NAMES;

  static std::vector<double> ROS_JOINT_UPPER_LIMITS;
  static std::vector<double> ROS_JOINT_LOWER_LIMITS;
};

} // namespace hardware
} // namespace crobot

