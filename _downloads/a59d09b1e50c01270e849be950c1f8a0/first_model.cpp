#include <cmath>
#include <iostream>
#include "crobot/kdl/rigid_body_tree.hpp"

int main() {
  using namespace crobot;
  auto robot = kdl::RigidBodyTree::MakeShared("first_model");
  auto shoulder = kdl::Joint::MakeShared("shoulder", kdl::JointType::REVOLUTE);
  shoulder->set_axis(math::Vector3d::UnitZ());
  shoulder->set_qpos_limits(-std::acos(-1.0), std::acos(-1.0));
  robot->add_body(kdl::RigidBody::MakeShared("arm", shoulder), "world");

  auto tool_joint = kdl::Joint::MakeShared("tool_fixed", kdl::JointType::FIXED);
  math::Matrix4d offset = math::Matrix4d::Identity();
  offset(0, 3) = 0.4;
  tool_joint->set_fixed_transform(offset);
  robot->add_body(kdl::RigidBody::MakeShared("tool", tool_joint), "arm");

  auto q = robot->get_home_qpos();
  std::cout << "Home tool position: "
            << robot->get_transform(q, "tool").block<3, 1>(0, 3).transpose()
            << '\n';
  q(robot->get_qpos_addr("shoulder").first) = std::acos(-1.0) / 2.0;
  std::cout << "Rotated tool position: "
            << robot->get_transform(q, "tool").block<3, 1>(0, 3).transpose()
            << "\nJacobian:\n" << robot->get_body_jacb(q, "tool") << '\n';
}
