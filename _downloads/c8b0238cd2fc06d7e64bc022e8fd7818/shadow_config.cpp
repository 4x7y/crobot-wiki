#include "crobot/hardware/shadow_config.hpp"

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "crobot/config.hpp"
#include "crobot/common/assert.hpp"
#include "crobot/hardware/shadow_finger.hpp"
#include "crobot/hardware/shadow_joint.hpp"
#include "crobot/math/constants.hpp"

using namespace crobot;
using namespace hardware;

static const std::string FILENAME_SHADOW_HAND_CONFIG
    = CROBOT_RESOURCE_LOCAL_PATH "shadow_description/config/config.yaml";

//==============================================================================
ShadowJoint::Ptr ShadowConfig::load_joint(YAML::Node node) {
  std::string name              = node["name"].as<std::string>();
  std::pair<int, int> qpos_addr = m_hand->get_joint(name)->get_qpos_addr()[0];
  CROBOT_ASSERT_ERROR(qpos_addr.second == 1);
  return std::make_shared<ShadowJoint>(name, qpos_addr.first);
}

//==============================================================================
ShadowMotor::Factor ShadowConfig::load_motor_factor(YAML::Node node) {
  ShadowMotor::Factor factor;
  auto joint_name = node["name"].as<std::string>();

  factor.joint   = get_joint(joint_name);
  factor.k_s_pos = node["k_s_pos"].as<double>();
  factor.k_s_neg = node["k_s_neg"].as<double>();
  return factor;
}

//==============================================================================
ShadowMotor::StrainGauge ShadowConfig::load_motor_strain_gauge(
    YAML::Node node) {
  ShadowMotor::StrainGauge strain_gauge;

  strain_gauge.index_pos_torque = node["index_pos_torque"].as<int>();
  strain_gauge.index_neg_torque = node["index_neg_torque"].as<int>();

  std::vector<ShadowMotor::Factor> factors;
  for (std::size_t i = 0; i < node["factor"].size(); ++i) {
    YAML::Node node_factor = node["factor"][i];
    factors.push_back(load_motor_factor(node_factor));
  }
  strain_gauge.factors = factors;
  return strain_gauge;
}

//==============================================================================
ShadowMotor::Control ShadowConfig::load_motor_control(YAML::Node node) {
  ShadowMotor::Control control;
  control.k = node["k"].as<double>();
  control.b = node["b"].as<double>();
  return control;
}

//==============================================================================
ShadowMotor::Ptr ShadowConfig::load_motor(YAML::Node node, std::size_t index) {
  std::vector<ShadowMotor::Factor> factors;

  ShadowMotor::Ptr motor = std::shared_ptr<ShadowMotor>(new ShadowMotor());
  motor->m_name          = node["name"].as<std::string>();
  motor->m_id            = index;
  motor->m_strain_gauge  = load_motor_strain_gauge(node["strain_gauge"]);
  motor->m_control       = load_motor_control(node["control"]);

  YAML::Node node_target = node["target"];
  for (std::size_t i = 0; i < node_target.size(); i++) {
    std::string joint_name      = node_target[i]["name"].as<std::string>();
    ShadowJoint::ConstPtr joint = get_joint(joint_name);

    YAML::Node node_range = node_target[i]["active_range"];
    std::pair<double, double> active_range
        = std::make_pair(-math::constantsd::inf(), math::constantsd::inf());
    if (node_range.IsDefined()) {
      active_range.first  = node_range[0].as<double>();
      active_range.second = node_range[1].as<double>();
    }
    motor->m_targets.push_back({joint, active_range});
  }

  return motor;
}

//==============================================================================
ShadowFinger::Ptr ShadowConfig::load_finger(YAML::Node node) {
  std::string name         = node["name"].as<std::string>();
  int id                   = node["id"].as<int>();
  std::string tip_bodyname = node["tip_bodyname"].as<std::string>();
  std::string tip_sitename = node["tip_sitename"].as<std::string>();

  ShadowFinger::Ptr finger = std::shared_ptr<ShadowFinger>(
      new ShadowFinger(m_hand, name, id, tip_bodyname, tip_sitename));

  for (std::size_t i = 0; i < node["joint"].size(); i++) {
    YAML::Node node_joint  = node["joint"][i];
    std::string joint_name = node_joint["name"].as<std::string>();
    finger->m_joints.push_back(get_joint(joint_name));
  }

  for (std::size_t i = 0; i < node["motor"].size(); i++) {
    YAML::Node node_motor  = node["motor"][i];
    std::string motor_name = node_motor["name"].as<std::string>();
    finger->m_motors.push_back(get_motor(motor_name));
    if (node_motor["active"].as<bool>()) {
      finger->m_motors_active.push_back(finger->m_motors.back());
    }
  }
  return finger;
}

//==============================================================================
ShadowConfig::ShadowConfig(kdl::ConstRigidBodyTreePtr hand) : m_hand(hand) {
  YAML::Node node_config = YAML::LoadFile(FILENAME_SHADOW_HAND_CONFIG);
  // Load joint
  YAML::Node node_joint = node_config["joint"];
  for (std::size_t i = 0; i < node_joint.size(); i++) {
    m_joints.push_back(load_joint(node_joint[i]));
  }

  // Load motor
  YAML::Node node_motor = node_config["motor"];
  for (std::size_t i = 0; i < node_motor.size(); i++) {
    m_motors.push_back(load_motor(node_motor[i], i));
  }

  // Load finger
  YAML::Node node_finger = node_config["finger"];
  for (std::size_t i = 0; i < node_finger.size(); i++) {
    m_fingers.push_back(load_finger(node_finger[i]));
  }
}

//==============================================================================
std::shared_ptr<ShadowFinger> ShadowConfig::get_finger(
    const std::string& finger_name) const {
  for (std::size_t i = 0; i < m_fingers.size(); i++) {
    if (finger_name == m_fingers[i]->get_name()) {
      return m_fingers[i];
    }
  }
  CROBOT_ASSERT_ERROR(false, "Finger %s not found.", finger_name.c_str());
  return nullptr;
}

//==============================================================================
ShadowMotor::Ptr ShadowConfig::get_motor(const std::string& motor_name) const {
  for (std::size_t i = 0; i < m_motors.size(); i++) {
    if (motor_name == m_motors[i]->get_name()) {
      return m_motors[i];
    }
  }
  CROBOT_ASSERT_ERROR(false, "Motor %s not found.", motor_name.c_str());
  return nullptr;
}

//==============================================================================
ShadowJoint::Ptr ShadowConfig::get_joint(const std::string& joint_name) const {
  for (std::size_t i = 0; i < m_joints.size(); i++) {
    if (joint_name == m_joints[i]->get_name()) {
      return m_joints[i];
    }
  }
  CROBOT_ASSERT_ERROR(false, "Joint %s not found.", joint_name.c_str());
  return nullptr;
}
