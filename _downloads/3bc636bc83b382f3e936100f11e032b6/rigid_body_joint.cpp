#include "crobot/kdl/rigid_body_joint.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/math/quaternion.hpp"
#include "crobot/math/random.hpp"
#include "crobot/math/rotation.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
JointPtr Joint::MakeShared(const std::string& name, JointType type) {
  return std::make_shared<Joint>(name, type);
}

//==============================================================================
Joint::Joint(const std::string& name, JointType type)
  : m_type(type), m_name(name), m_in_tree(false) {
  m_tform_joint2parent = math::Matrix4d::Identity();
  m_tform_child2joint  = math::Matrix4d::Identity();

  if (m_type == JointType::FIXED) {
    m_axis            = math::Vector3d::Zero();
    m_limits          = std::make_pair<>(math::Vector1d(0), math::Vector1d(0));
    m_home_position   = math::Vector1d(0);
    m_motion_subspace = math::Vector6d::Zero();
  }
  else if (m_type == JointType::REVOLUTE) {
    m_axis   = math::Vector3d(0, 0, 1);
    m_limits = std::make_pair<>(math::Vector1d(-M_PI), math::Vector1d(M_PI));
    m_home_position   = math::Vector1d(0);
    m_motion_subspace = math::Vector6d(0, 0, 1, 0, 0, 0);
  }
  else if (m_type == JointType::PRISMATIC) {
    m_axis   = math::Vector3d::Zero();
    m_limits = std::make_pair<>(
        math::Vector1d(-10.0),
        math::Vector1d(10.0)); // free joint default limit
    m_home_position   = math::Vector1d(0);
    m_motion_subspace = math::Vector6d(0, 0, 0, 0, 0, 1);
  }
  else if (m_type == JointType::BALL) {
    m_home_position = math::Vector4d(1, 0, 0, 0);
    m_limits        = std::make_pair<>(
        math::Vector4d::Ones() * -1.0, math::Vector4d::Ones());
    m_motion_subspace            = math::Matrix63::Zero();
    m_motion_subspace.topRows(3) = math::Matrix3d::Identity();
  }
}

//==============================================================================
void Joint::set_name(const std::string& name) {
  m_name = name;
}

//==============================================================================
const std::string& Joint::get_name() const {
  return m_name;
}

//==============================================================================
JointType Joint::get_type() const {
  return m_type;
}

//==============================================================================
int Joint::get_qpos_dim() const {
  switch (m_type) {
    case JointType::FIXED:
      return 0;
    case JointType::PRISMATIC:
      return 1;
    case JointType::REVOLUTE:
      return 1;
    case JointType::BALL:
      return 4;
  }
  return -1;
}

//==============================================================================
int Joint::get_qvel_dim() const {
  switch (m_type) {
    case JointType::FIXED:
      return 0;
    case JointType::PRISMATIC:
      return 1;
    case JointType::REVOLUTE:
      return 1;
    case JointType::BALL:
      return 3;
  }
  return -1;
}

//==============================================================================
std::vector<std::pair<int, int>> Joint::get_qpos_addr() const {
  return {m_qpos_addr};
}

//==============================================================================
std::vector<std::pair<int, int>> Joint::get_qvel_addr() const {
  return {m_qvel_addr};
}

//==============================================================================
std::vector<double> Joint::get_transmission_coeffs() const {
  return {1};
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> Joint::get_qpos_limits() const {
  return m_limits;
}

//==============================================================================
void Joint::set_qpos_limits(double lb, double ub) {
  CROBOT_ASSERT_ERROR(
      m_type == JointType::PRISMATIC || m_type == JointType::REVOLUTE);
  set_qpos_limits(math::Vector1d(lb), math::Vector1d(ub));
}

//==============================================================================
void Joint::set_qpos_limits(
    const math::VectorXd& lb, const math::VectorXd& ub) {
  m_limits = std::make_pair<>(lb, ub);
}

//==============================================================================
void Joint::set_home_position(double qpos) {
  CROBOT_ASSERT_ERROR(
      m_type == JointType::PRISMATIC || m_type == JointType::REVOLUTE,
      "Only revolute or prismatic joints have scalar position value.");
  m_home_position = math::Vector1d(qpos);
}

//==============================================================================
void Joint::set_home_position(const math::VectorXd& qpos) {
  CROBOT_ASSERT_ERROR(
      qpos.size() == get_qpos_dim(),
      "The qpos dimension does not match the joint's position number.");
  m_home_position = qpos;
}

//==============================================================================
math::VectorXd Joint::random_qpos() const {
  CROBOT_ASSERT_ERROR(m_type != JointType::FIXED);
  math::VectorXd qpos_rand(get_qpos_dim());
  if (m_type == JointType::BALL) {
    math::Quaternion quat_rand = math::Quaternion::UnitRandom();
    qpos_rand(0)               = quat_rand.w();
    qpos_rand.segment(1, 3)    = quat_rand.vec();
  }
  else if (m_type == JointType::PRISMATIC || m_type == JointType::REVOLUTE) {
    qpos_rand = math::Random::uniform(m_limits.first, m_limits.second);
  }
  else {
    CROBOT_LOG_ERROR("Unknown joint type.");
  }
  return qpos_rand;
}

//==============================================================================
math::VectorXd Joint::random_qpos_offset(
    double ref, double offset_limit) const {
  CROBOT_ASSERT_ERROR(m_type != JointType::FIXED);
  offset_limit = std::abs(offset_limit);
  math::VectorXd qpos_rand(get_qpos_dim());
  if (m_type == JointType::REVOLUTE && qpos_rand.size() == 1) {
    qpos_rand << math::Random::uniform(
        std::max(m_limits.first[0], ref - offset_limit),
        std::min(m_limits.second[0], ref + offset_limit));
    return qpos_rand;
  }
  else {
    return random_qpos();
  }
}

//==============================================================================
void Joint::set_axis(const math::Vector3d& axis) {
  CROBOT_ASSERT_ERROR(
      m_type == JointType::PRISMATIC || m_type == JointType::REVOLUTE);
  m_axis = axis.normalized();
  if (m_type == JointType::REVOLUTE) {
    m_motion_subspace
        = math::Vector6d(m_axis(0), m_axis(1), m_axis(2), 0, 0, 0);
  }
  else if (m_type == JointType::PRISMATIC) {
    m_motion_subspace
        = math::Vector6d(0, 0, 0, m_axis(0), m_axis(1), m_axis(2));
  }
}

//==============================================================================
void Joint::set_fixed_transform(const math::Matrix4d& tform) {
  m_tform_joint2parent = tform;
  m_tform_child2joint  = math::Matrix4d::Identity();
}

//==============================================================================
math::Matrix4d Joint::tform_body2parent() const {
  CROBOT_ASSERT_ERROR(m_type == JointType::FIXED);
  return m_tform_joint2parent * m_tform_child2joint;
}

//==============================================================================
math::Matrix4d Joint::tform_body2parent(double qpos) const {
  CROBOT_ASSERT_ERROR(
      m_type == JointType::PRISMATIC || m_type == JointType::REVOLUTE);
  return m_tform_joint2parent * tform_joint(math::Vector1d(qpos))
         * m_tform_child2joint;
}

//==============================================================================
math::Matrix4d Joint::tform_body2parent(const math::VectorXd& qpos) const {
  return m_tform_joint2parent * tform_joint(qpos) * m_tform_child2joint;
}

//==============================================================================
math::Matrix4d Joint::tform_joint(const math::VectorXd& qpos) const {
  if (m_type == JointType::FIXED) {
    return math::Matrix4d::Identity();
  }
  else if (m_type == JointType::REVOLUTE) {
    math::AngleAxisd aaxis  = math::AngleAxisd(qpos(0), m_axis);
    math::Matrix3d rotm     = aaxis.toRotationMatrix();
    math::Matrix4d tform    = math::Matrix4d::Identity();
    tform.block<3, 3>(0, 0) = rotm;
    return tform;
  }
  else if (m_type == JointType::PRISMATIC) {
    math::Matrix4d tform    = math::Matrix4d::Identity();
    tform.block<3, 1>(0, 3) = m_axis * qpos(0);
    return tform;
  }
  else if (m_type == JointType::BALL) {
    // qpos = [w, x, y, z]
    math::Vector4d qpos_nm = qpos.normalized();
    math::Quaternion quat
        = math::Quaternion(qpos_nm(0), qpos_nm(1), qpos_nm(2), qpos_nm(3));
    math::Matrix3d rotm     = quat.toRotationMatrix();
    math::Matrix4d tform    = math::Matrix4d::Identity();
    tform.block<3, 3>(0, 0) = rotm;
    return tform;
  }
  else {
    CROBOT_LOG_ERROR("Unknown joint type.");
  }
  return math::Matrix4d::Identity();
}
