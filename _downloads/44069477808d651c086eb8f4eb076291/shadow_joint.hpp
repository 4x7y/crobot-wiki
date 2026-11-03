#pragma once

#include <memory>
#include <string>

#include "crobot/math/matrix.hpp"

namespace crobot::hardware {

class ShadowJoint {
public:
  using Ptr      = std::shared_ptr<ShadowJoint>;
  using ConstPtr = std::shared_ptr<const ShadowJoint>;

  ShadowJoint(const std::string& name, int id);

  /**
   * @brief Get the joint name.
   *
   * @return const std::string& Name of the joint
   */
  const std::string& get_name() const;

  /**
   * @brief Get the qpos addr object
   *
   * @return int
   */
  int get_qpos_addr() const;

private:
  std::string m_name;
  int m_id;
};

} // namespace crobot::hardware

