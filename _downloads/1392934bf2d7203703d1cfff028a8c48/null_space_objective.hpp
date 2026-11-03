#pragma once

#include <functional>

#include "crobot/ik/obj_target.hpp"
#include "crobot/kdl/kdl.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ik {
class NullSpaceObj {
public:
  using Ptr      = std::shared_ptr<NullSpaceObj>;
  using ConstPtr = std::shared_ptr<const NullSpaceObj>;
  NullSpaceObj(kdl::ConstRigidBodyTreePtr rbtree);

  ~NullSpaceObj() = default;

  void load_from_file(const std::string& filename);

  math::VectorXd get_weighted_partial(
      math::VectorXd qpos, const math::VectorXd& qvel) const;

  void add_jnt_limit_target(
      const std::vector<std::string>& joint_names, double weight);

  void add_jnt_vel_target(double weight);

  void add_jnt_vel_target(std::vector<std::string> joint_names, double weight);

  void add_distance_target(
      const std::string& body_name1,
      const math::Vector3d& bpos1,
      const std::string& body_name2,
      const math::Vector3d& bpos2,
      double checking_threshold,
      double weight);

  void add_quat_target(
      const std::string& body_name,
      const math::Quaternion& quat_target,
      double weight);

private:
  kdl::ConstRigidBodyTreePtr m_rbtree;

  std::vector<ObjTarget::Ptr> m_obj_targets;
};

} // namespace crobot::ik

