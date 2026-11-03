#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "crobot/common/vector_mapper.hpp"
#include "crobot/hardware/shadow_finger.hpp"
#include "crobot/hardware/shadow_joint.hpp"
#include "crobot/hardware/shadow_motor.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::hardware {

class ShadowConfig {
public:
  using Ptr      = std::shared_ptr<ShadowConfig>;
  using ConstPtr = std::shared_ptr<const ShadowConfig>;

  ShadowConfig(kdl::ConstRigidBodyTreePtr hand);

  /**
   * @brief Get the finger names
   *
   * @return const std::vector<std::string>&
   */
  const std::vector<std::string>& get_finger_names() const;

  /**
   * @brief Convert the finger name to the finger index.
   *
   * @param finger_name Name of the finger
   * @return std::size_t The finger index
   */
  std::size_t finger_name2index(const std::string& finger_name) const;

  /**
   * @brief Load a finger config from the file.
   *
   * @param finger_name
   * @return std::shared_ptr<ShadowFinger>
   */
  ShadowFinger::Ptr get_finger(const std::string& finger_name) const;

  /**
   * @brief Get the motor object by the motor name.
   *
   * @param motor_name Name of the motor
   * @return ShadowMotor::Ptr The motor object
   */
  ShadowMotor::Ptr get_motor(const std::string& motor_name) const;

  /**
   * @brief Get the joint object by the joint name.
   *
   * @param joint_name Name of the joint
   * @return ShadowMotor::Ptr The motor object
   */
  ShadowJoint::Ptr get_joint(const std::string& joint_name) const;

  /**
   * @brief Reorder joint configuration values from the ROS order to CROBOT
   * order.
   *
   * @param qpos_ros The joint configuration values in the ROS order.
   * @return math::VectorXd The joint configuration values in the CROBOT order.
   */
  math::VectorXd qpos_ros2crobot(const math::VectorXd& qpos_ros);

private:
  ShadowJoint::Ptr load_joint(YAML::Node node);

  ShadowMotor::Factor load_motor_factor(YAML::Node node);

  ShadowMotor::StrainGauge load_motor_strain_gauge(YAML::Node node);

  ShadowMotor::Control load_motor_control(YAML::Node node);

  ShadowMotor::Ptr load_motor(YAML::Node node, std::size_t index);

  ShadowFinger::Ptr load_finger(YAML::Node node);

  kdl::ConstRigidBodyTreePtr m_hand;
  std::vector<ShadowJoint::Ptr> m_joints;
  std::vector<ShadowMotor::Ptr> m_motors;
  std::vector<ShadowFinger::Ptr> m_fingers;
};

} // namespace crobot::hardware

