#pragma once

#include <mujoco/mujoco.h>

#include "crobot/kdl/rigid_body_joint.hpp"

namespace crobot::simulation {

// Forward declaration
class MJJoint;
using MJJointPtr      = std::shared_ptr<MJJoint>;
using ConstMJJointPtr = std::shared_ptr<const MJJoint>;
class MJRBTree;
using ConstMJRBTreePtr = std::shared_ptr<const MJRBTree>;

class MJJoint : public kdl::Joint {
public:
  static MJJointPtr MakeShared(const mjModel* model, int jid_mujoco);

  static MJJointPtr MakeSharedDummy(
      const mjModel* model,
      const std::string& name,
      kdl::JointType type,
      int qpos_addr,
      int qvel_addr);

  static MJJointPtr MakeSharedFixed(const mjModel* model, int bid_mujoco);

  MJJoint(const mjModel* model, int jid_mujoco);

  MJJoint(
      const mjModel* model,
      const std::string& name,
      kdl::JointType type,
      int qpos_addr,
      int qvel_addr);

  MJJoint(const mjModel* model, const std::string& name, kdl::JointType type);

  int get_id() const;

  using kdl::Joint::get_name;

  bool is_limited() const;

private:
  static std::string get_name(const mjModel* model, int jid_mujoco);

  static kdl::JointType get_type(const mjModel* model, int jid_mujoco);

  const mjModel* m_model;
  int m_mjid;

  ConstMJRBTreePtr m_rbtree;
  std::pair<int, int> m_qpos_addr;
  std::pair<int, int> m_qvel_addr;

  friend class MJRBTree;
};

} // namespace crobot::simulation

