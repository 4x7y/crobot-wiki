#include "crobot/hardware/shadow_sensor_profile.hpp"

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace hardware;

//==============================================================================
ShadowSensorProfile::ShadowSensorProfile(const std::string& filename) {
  YAML::Node node = YAML::LoadFile(filename);
  CROBOT_ASSERT_ERROR(
      node.IsDefined(), "Failed to load YAML file: %s", filename.c_str());

  YAML::Node node_shadow = node["shadow_hand"];
  CROBOT_ASSERT_ERROR(
      node_shadow.IsDefined(), "Failed to find shadow_hand tag.");

  for (const auto& motor_node : node_shadow["motor_names"]) {
    m_motor_names.push_back(motor_node.as<std::string>());
  }
  for (const auto& joint_node : node_shadow["joint_names"]) {
    m_joint_names.push_back(joint_node.as<std::string>());
  }

  YAML::Node node_adept = node["adept"];
  if (node_adept.IsDefined()) {
    std::vector<double> adept_xpos
        = node["adept"]["xpos"].as<std::vector<double>>();
    m_adept_xpos = {adept_xpos[0], adept_xpos[1], adept_xpos[2], adept_xpos[3]};
  }
  else {
    m_adept_xpos = {0, 0, 0, 0};
  }

  YAML::Node node_data = node_shadow["data"];
  m_data.resize(node_data.size());
  for (size_t i = 0; i < node_data.size(); i++) {

    // qpos_actual
    auto qpos_actual = node_data[i]["qpos_actual"].as<std::vector<double>>();
    m_data[i].qpos_actual
        = Eigen::Map<math::VectorXd>(qpos_actual.data(), qpos_actual.size());

    // qpos_command
    auto qpos_command = node_data[i]["qpos_command"].as<std::vector<double>>();
    m_data[i].qpos_command
        = Eigen::Map<math::VectorXd>(qpos_command.data(), qpos_command.size());

    // effort
    auto effort      = node_data[i]["effort"].as<std::vector<double>>();
    m_data[i].effort = Eigen::Map<math::VectorXd>(effort.data(), effort.size());

    // strain_gauge_left
    auto strain_gauge_left
        = node_data[i]["strain_gauge_left"].as<std::vector<double>>();
    m_data[i].strain_gauge_left = Eigen::Map<math::VectorXd>(
        strain_gauge_left.data(), strain_gauge_left.size());

    // strain_gauge_right
    auto strain_gauge_right
        = node_data[i]["strain_gauge_right"].as<std::vector<double>>();
    m_data[i].strain_gauge_right = Eigen::Map<math::VectorXd>(
        strain_gauge_right.data(), strain_gauge_right.size());

    // ft_sensor
    auto xfrc           = node_data[i]["ft_sensor"].as<std::vector<double>>();
    m_data[i].ft_sensor = math::Vector3d(xfrc[0], xfrc[1], xfrc[2]);
  }
}

//==============================================================================
std::size_t ShadowSensorProfile::size() const {
  return m_data.size();
}

//==============================================================================
math::VectorXd ShadowSensorProfile::get_qpos_actual(std::size_t index) const {
  return m_data[index].qpos_actual;
}

//==============================================================================
math::VectorXd ShadowSensorProfile::get_qpos_command(std::size_t index) const {
  return m_data[index].qpos_command;
}

//==============================================================================
math::VectorXd ShadowSensorProfile::get_effort(std::size_t index) const {
  return m_data[index].effort;
}

//==============================================================================
math::Vector3d ShadowSensorProfile::get_xfrc(std::size_t index) const {
  return m_data[index].ft_sensor;
}

//==============================================================================
math::VectorXd ShadowSensorProfile::get_strain_gauge_left(
    std::size_t index) const {
  return m_data[index].strain_gauge_left;
}

//==============================================================================
math::VectorXd ShadowSensorProfile::get_strain_gauge_right(
    std::size_t index) const {
  return m_data[index].strain_gauge_right;
}

//==============================================================================
math::Vector4d ShadowSensorProfile::get_adept_xpos() const {
  return m_adept_xpos;
}

//==============================================================================
std::vector<std::string> ShadowSensorProfile::get_joint_names() const {
  return m_joint_names;
}

//==============================================================================
std::size_t ShadowSensorProfile::get_motor_id(std::string motor_name) const {
  for (size_t i = 0; i < m_motor_names.size(); i++) {
    if (m_motor_names[i] == motor_name) {
      return i;
    }
  }

  CROBOT_ASSERT_ERROR(
      false, "Failed to find motor name: %s", motor_name.c_str());
  return m_motor_names.size();
}
