#include "crobot/hardware/shadow_tension_friction.hpp"

#include <numeric>

#include <experimental/filesystem>
#include <yaml-cpp/yaml.h>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace hardware;

//==============================================================================
void TensionFriction::load_file(const std::string& filename) {
  YAML::Node node = YAML::LoadFile(filename);

  // load joint info
  for (size_t i = 0; i < node["joint"].size(); i++) {
    JointSampleInfo info;
    info.name         = node["joint"][i]["name"].as<std::string>();
    info.lb           = node["joint"][i]["joint_angle_lb"].as<double>();
    info.ub           = node["joint"][i]["joint_angle_ub"].as<double>();
    info.num_interval = node["joint"][i]["joint_angle_intervals"].as<int>();
    info.angle_interval
        = (info.ub - info.lb) / static_cast<double>(info.num_interval);
    m_joints.push_back(info);
  }

  // set stride for joints
  std::size_t stride = 1;
  for (size_t i = 0; i < m_joints.size(); i++) {
    m_joints[m_joints.size() - i - 1].stride = stride;
    stride = stride * (m_joints[m_joints.size() - i - 1].num_interval + 1);
  }

  // load actuator info
  for (size_t i = 0; i < node["actuator"].size(); i++) {
    m_actuators.insert({{node["actuator"][i]["name"].as<std::string>(), i}});
  }

  // compute the number of data points
  std::size_t num_data = 1;
  if (m_joints.empty()) {
    num_data = 0;
  }
  else {
    num_data = 1;
    for (size_t i = 0; i < m_joints.size(); i++) {
      num_data = num_data * (m_joints[i].num_interval + 1);
    }
  }
  m_tension_friction.resize(num_data);

  // load data
  for (size_t i = 0; i < node["data"].size(); i++) {
    auto node_data_i = node["data"][i];
    std::vector<std::size_t> joint_positions
        = node_data_i["joint_positions"].as<std::vector<std::size_t>>();
    std::size_t index = compute_index(joint_positions);

    std::vector<double> effort_inc
        = node_data_i["effort_inc"].as<std::vector<double>>();
    m_tension_friction[index].first
        = Eigen::Map<Eigen::VectorXd>(effort_inc.data(), effort_inc.size());
    std::vector<double> effort_dec
        = node_data_i["effort_dec"].as<std::vector<double>>();
    m_tension_friction[index].second
        = Eigen::Map<Eigen::VectorXd>(effort_dec.data(), effort_dec.size());
  }
}

//==============================================================================
math::VectorXd TensionFriction::get_friction(const math::VectorXd& qpos) const {
  std::vector<std::pair<std::size_t, std::size_t>> indices
      = get_joint_angle_indices(qpos);
  std::size_t num_points = std::pow(2, indices.size());
  std::vector<math::VectorXd> torques;
  torques.reserve(num_points);
  std::vector<double> distances;
  distances.reserve(num_points);

  for (size_t i = 0; i < num_points; i++) {
    std::vector<std::size_t> joint_positions;
    for (size_t j = 0; j < indices.size(); j++) {
      if (((i >> j) & 1U) == 0) {
        joint_positions.push_back(indices[j].first);
      }
      else {
        joint_positions.push_back(indices[j].second);
      }
    }

    std::size_t index = compute_index(joint_positions);
    math::VectorXd torque_friction
        = (m_tension_friction[index].first - m_tension_friction[index].second)
          / 2.;
    torques.push_back(torque_friction);

    double distance = distance_to_joint_positions(qpos, joint_positions);
    if (distance < math::constantsd::epsilon()) {
      return torques.back();
    }
    distances.push_back(distance);
  }

  // compute weight for averaging the frictional torques
  std::vector<double> weights;
  weights.reserve(distances.size());
  for (size_t i = 0; i < distances.size(); i++) {
    weights.push_back(1.0 / distances[i]);
  }
  double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);

  // averaged torque
  math::VectorXd torque_avg = math::VectorXd::Zero(m_actuators.size());
  for (size_t i = 0; i < torques.size(); i++) {
    torque_avg += weights[i] / weight_sum * torques[i];
  }

  return torque_avg;
}

//==============================================================================
std::vector<std::pair<std::size_t, std::size_t>>
TensionFriction::get_joint_angle_indices(const math::VectorXd& qpos) const {
  CROBOT_ASSERT_ERROR(qpos.size() == static_cast<int>(m_joints.size()));
  std::vector<std::pair<std::size_t, std::size_t>> indices;
  indices.reserve(m_joints.size());
  for (int i = 0; i < qpos.size(); i++) {
    if (qpos(i) <= m_joints[i].lb) {
      indices.push_back(std::make_pair<>(0, 0));
    }
    else if (qpos(i) >= m_joints[i].ub) {
      indices.push_back(
          std::make_pair<>(m_joints[i].num_interval, m_joints[i].num_interval));
    }
    else {
      std::size_t index_lb = static_cast<std::size_t>(
          (qpos(i) - m_joints[i].lb) / m_joints[i].angle_interval);
      std::size_t index_ub = index_lb + 1;
      indices.push_back(std::make_pair<>(index_lb, index_ub));
    }
  }

  return indices;
}

//==============================================================================
double TensionFriction::distance_to_joint_positions(
    const math::VectorXd& qpos,
    const std::vector<std::size_t>& joint_positions) const {
  CROBOT_ASSERT_ERROR(qpos.size() == static_cast<int>(m_joints.size()));
  CROBOT_ASSERT_ERROR(joint_positions.size() == m_joints.size());
  math::VectorXd qpos_target(qpos.size());
  for (int i = 0; i < qpos.size(); i++) {
    qpos_target(i)
        = static_cast<double>(joint_positions[i]) * m_joints[i].angle_interval
          + m_joints[i].lb;
  }
  return (qpos - qpos_target).norm();
}

//==============================================================================
std::size_t TensionFriction::compute_index(
    const std::vector<std::size_t>& joint_poisitions) const {
  CROBOT_ASSERT_ERROR(joint_poisitions.size() == m_joints.size());
  std::size_t index = 0;
  for (size_t i = 0; i < m_joints.size(); i++) {
    index = index + m_joints[i].stride * joint_poisitions[i];
  }
  return index;
}
