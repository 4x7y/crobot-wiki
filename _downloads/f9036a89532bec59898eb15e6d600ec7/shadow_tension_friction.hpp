#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "crobot/math/matrix.hpp"

namespace crobot::hardware {

class TensionFriction {
public:
  using Ptr      = std::shared_ptr<TensionFriction>;
  using ConstPtr = std::shared_ptr<const TensionFriction>;

  TensionFriction() = default;

  void load_file(const std::string& filename);

  math::VectorXd get_friction(const math::VectorXd& qpos) const;

private:
  struct JointSampleInfo
  {
    std::string name;
    double lb;
    double ub;
    double angle_interval;
    int num_interval;
    std::size_t stride;
  };

  std::size_t compute_index(
      const std::vector<std::size_t>& joint_poisitions) const;

  std::vector<std::pair<std::size_t, std::size_t>> get_joint_angle_indices(
      const math::VectorXd& qpos) const;

  double distance_to_joint_positions(
      const math::VectorXd& qpos,
      const std::vector<std::size_t>& joint_positions) const;

  std::vector<std::pair<math::VectorXd, math::VectorXd>> m_tension_friction;
  std::vector<JointSampleInfo> m_joints;
  std::unordered_map<std::string, std::size_t> m_actuators;
};

} // namespace crobot::hardware

