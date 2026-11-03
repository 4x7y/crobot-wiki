#include "crobot/simulation/mj_joint.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/kdl/rigid_body_joint.hpp"
#include "crobot/math/quaternion.hpp"

using namespace crobot;
using namespace simulation;

//==============================================================================
MJJointPtr MJJoint::MakeShared(const mjModel* model, int id) {
  return std::make_shared<MJJoint>(model, id);
}

//==============================================================================
MJJointPtr MJJoint::MakeSharedDummy(
    const mjModel* model,
    const std::string& name,
    kdl::JointType type,
    int qpos_addr,
    int qvel_addr) {
  return std::make_shared<MJJoint>(model, name, type, qpos_addr, qvel_addr);
}

//==============================================================================
MJJointPtr MJJoint::MakeSharedFixed(const mjModel* model, int bid) {
  CROBOT_ASSERT_ERROR(model != nullptr, "Invalid Mujoco model pointer.");
  CROBOT_ASSERT_ERROR(
      model->body_jntnum[bid] == 0,
      "The specified body should have no joints.");
  std::string joint_name = [&]() {
    const char* body_name = mj_id2name(model, mjOBJ_BODY, bid);
    if (body_name == nullptr) {
      CROBOT_LOG_WARN("body {} is not associated with a name.", bid);
      return "body_" + std::to_string(bid) + "_fixed_joint";
    }
    else {
      return std::string(body_name) + "_fixed_joint";
    }
  }();

  MJJointPtr joint
      = std::make_shared<MJJoint>(model, joint_name, kdl::JointType::FIXED);

  math::Vector3d body_pos = math::Vector3d::Map(model->body_pos + bid * 3);
  math::Quaternion body_quat(
      model->body_quat[bid * 4 + 0],
      model->body_quat[bid * 4 + 1],
      model->body_quat[bid * 4 + 2],
      model->body_quat[bid * 4 + 3]);
  math::Matrix4d tform_child2parent    = math::Matrix4d::Identity();
  tform_child2parent.block<3, 3>(0, 0) = body_quat.as_rotm();
  tform_child2parent.block<3, 1>(0, 3) = body_pos;

  // body frame to joint frame
  joint->m_tform_child2joint = math::Matrix4d::Identity();
  // joint frame to parent link's body frame
  joint->m_tform_joint2parent = tform_child2parent;

  return joint;
}

//==============================================================================
MJJoint::MJJoint(const mjModel* model, int mjid)
  : kdl::Joint(get_name(model, mjid), get_type(model, mjid)),
    m_model(model),
    m_mjid(mjid),
    m_rbtree(nullptr) {
  CROBOT_ASSERT_ERROR(m_model != nullptr, "Invalid Mujoco model pointer.");
  CROBOT_ASSERT_ERROR(
      0 <= m_mjid && m_mjid < m_model->njnt, "Invalid joint id.");

  // joint properties from Mujoco model
  int joint_type = m_model->jnt_type[m_mjid];
  int bid        = m_model->jnt_bodyid[m_mjid];

  // Set joint axis for PRISMATIC and REVOLUTE joints
  if (m_type == kdl::JointType::PRISMATIC) {
    set_axis(math::Vector3d::Map(m_model->jnt_axis + m_mjid * 3));
  }
  else if (m_type == kdl::JointType::REVOLUTE) {
    set_axis(math::Vector3d::Map(m_model->jnt_axis + m_mjid * 3));
  }

  // If this joint is the first joint of the corresponding body
  if (m_model->body_jntadr[bid] == m_mjid) {
    math::Matrix4d tform_child2parent;
    math::Matrix4d tform_joint2child;

    math::Vector3d body_pos = math::Vector3d::Map(m_model->body_pos + bid * 3);
    math::Quaternion body_quat(
        m_model->body_quat[bid * 4 + 0],
        m_model->body_quat[bid * 4 + 1],
        m_model->body_quat[bid * 4 + 2],
        m_model->body_quat[bid * 4 + 3]);
    tform_child2parent                   = math::Matrix4d::Identity();
    tform_child2parent.block<3, 3>(0, 0) = body_quat.as_rotm();
    tform_child2parent.block<3, 1>(0, 3) = body_pos;

    math::Vector3d jnt_pos = math::Vector3d::Map(m_model->jnt_pos + m_mjid * 3);
    tform_joint2child      = math::Matrix4d::Identity();
    tform_joint2child.block<3, 1>(0, 3) = jnt_pos;

    // body frame to joint frame
    m_tform_child2joint = tform_joint2child.tform_inv();
    // joint frame to parent link's body frame
    m_tform_joint2parent = tform_child2parent * tform_joint2child;
  }
  else {
    math::Matrix4d tform_child2parent = math::Matrix4d::Identity();

    math::Vector3d jnt_pos = math::Vector3d::Map(m_model->jnt_pos + m_mjid * 3);
    math::Matrix4d tform_joint2child    = math::Matrix4d::Identity();
    tform_joint2child.block<3, 1>(0, 3) = jnt_pos;

    // body frame to joint frame
    m_tform_child2joint = tform_joint2child.tform_inv();
    // joint frame to parent link's body frame
    m_tform_joint2parent = tform_child2parent * tform_joint2child;
  }

  // qpos addr
  int qpos_addr = m_model->jnt_qposadr[m_mjid];
  int ndim      = 0;
  if (joint_type == mjJNT_BALL)
    ndim = 4;
  else if (joint_type == mjJNT_HINGE || joint_type == mjJNT_SLIDE)
    ndim = 1;
  else
    CROBOT_ASSERT_ERROR(false, "Joint type not defined.");
  m_qpos_addr = std::make_pair(qpos_addr, ndim);

  // qvel addr
  int qvel_addr = m_model->jnt_dofadr[m_mjid];
  int ndof      = 0;
  if (joint_type == mjJNT_BALL)
    ndof = 3;
  else if (joint_type == mjJNT_HINGE || joint_type == mjJNT_SLIDE)
    ndof = 1;
  else
    CROBOT_ASSERT_ERROR(false, "Joint type not defined.");
  m_qvel_addr = std::make_pair(qvel_addr, ndof);

  // joint limits
  if (joint_type == mjJNT_HINGE || joint_type == mjJNT_SLIDE) {
    double jnt_lb = m_model->jnt_range[m_mjid * 2 + 0];
    double jnt_ub = m_model->jnt_range[m_mjid * 2 + 1];
    set_qpos_limits(jnt_lb, jnt_ub);
  }
}

//==============================================================================
MJJoint::MJJoint(
    const mjModel* model,
    const std::string& name,
    kdl::JointType type,
    int qpos_addr,
    int qvel_addr)
  : kdl::Joint(name, type), m_model(model), m_mjid(-1), m_rbtree(nullptr) {
  CROBOT_ASSERT_ERROR(m_model != nullptr, "Invalid Mujoco model pointer.");
  CROBOT_ASSERT_ERROR(
      m_type == kdl::JointType::PRISMATIC || m_type == kdl::JointType::REVOLUTE
          || m_type == kdl::JointType::BALL,
      "Dummy joint type should be PRSIMARTIC, REVOLUTE, or BALL.");
  if (m_type == kdl::JointType::PRISMATIC
      || m_type == kdl::JointType::REVOLUTE) {
    m_qpos_addr = std::make_pair<>(qpos_addr, 1);
    m_qvel_addr = std::make_pair<>(qvel_addr, 1);
  }
  else if (m_type == kdl::JointType::BALL) {
    m_qpos_addr = std::make_pair<>(qpos_addr, 4);
    m_qvel_addr = std::make_pair<>(qvel_addr, 3);
  }
  else {
    CROBOT_LOG_ERROR("Should not go to this line.");
  }
}

//==============================================================================
MJJoint::MJJoint(
    const mjModel* model, const std::string& name, kdl::JointType type)
  : kdl::Joint(name, type), m_model(model), m_mjid(-1), m_rbtree(nullptr) {
  m_qpos_addr = std::make_pair<>(0, -1);
  m_qvel_addr = std::make_pair<>(0, -1);
}

//==============================================================================
int MJJoint::get_id() const {
  return m_mjid;
}

//==============================================================================
std::string MJJoint::get_name(const mjModel* model, int id) {
  char* name = model->names + model->name_jntadr[id];
  return std::string(name);
}

//==============================================================================
kdl::JointType MJJoint::get_type(const mjModel* model, int id) {
  switch (model->jnt_type[id]) {
    case mjJNT_FREE:
      CROBOT_LOG_ERROR(
          "Mujoco FREE Joint should be mapped to three PRISMATIC joints and a "
          "following BALL joint.");
      break;

    case mjJNT_BALL:
      return kdl::JointType::BALL;
      break;

    case mjJNT_HINGE:
      return kdl::JointType::REVOLUTE;
      break;

    case mjJNT_SLIDE:
      return kdl::JointType::PRISMATIC;
      break;

    default:
      CROBOT_LOG_ERROR("Unknown Mujoco joint type.");
      break;
  }
  // should not get to this line.
  return kdl::JointType::FIXED;
}
