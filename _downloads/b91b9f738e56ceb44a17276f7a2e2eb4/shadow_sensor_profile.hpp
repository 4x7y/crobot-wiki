#pragma once

#include <string>
#include <vector>

#include "crobot/math/matrix.hpp"

namespace crobot::hardware {

class ShadowSensorProfile {
public:
  ShadowSensorProfile(const std::string& filename);

  std::size_t size() const;

  math::VectorXd get_qpos_actual(std::size_t index) const;

  math::VectorXd get_qpos_command(std::size_t index) const;

  math::VectorXd get_effort(std::size_t index) const;

  math::Vector3d get_xfrc(std::size_t index) const;

  math::VectorXd get_strain_gauge_left(std::size_t index) const;

  math::VectorXd get_strain_gauge_right(std::size_t index) const;

  math::Vector4d get_adept_xpos() const;

  std::vector<std::string> get_joint_names() const;

  std::size_t get_motor_id(std::string motor_name) const;

private:
  struct Data
  {
    math::VectorXd qpos_actual;
    math::VectorXd qpos_command;
    math::VectorXd strain_gauge_left;
    math::VectorXd strain_gauge_right;
    math::VectorXd effort;
    math::Vector3d ft_sensor;
  };

  std::vector<std::string> m_motor_names;
  std::vector<std::string> m_joint_names;
  math::Vector4d m_adept_xpos;
  std::vector<Data> m_data;
};

} // namespace crobot::hardware

