#include "crobot/ik/null_space_objective.hpp"

#include "crobot/common/common.hpp"
#include "crobot/ik/obj_distance_target.hpp"
#include "crobot/ik/obj_jnt_limit_target.hpp"
#include "crobot/ik/obj_jnt_vel_target.hpp"
#include "crobot/ik/obj_quat_target.hpp"

using namespace crobot;
using namespace kdl;
using namespace ik;

//==============================================================================
NullSpaceObj::NullSpaceObj(ConstRigidBodyTreePtr rbtree) : m_rbtree(rbtree) {
}

//==============================================================================
void NullSpaceObj::load_from_file(const std::string& filename) {
  YAML::Node yaml = YAML::LoadFile(filename)["objectives"];

  for (const auto& obj : yaml) {
    std::string obj_type = obj["type"].as<std::string>();
    if (obj_type == "JNT_LIMIT") {
      double weight = obj["weight"].as<double>();

      if (obj["joint_names"].IsDefined()) {
        std::vector<std::string> joint_names
            = obj["joint_names"].as<std::vector<std::string>>();
        add_jnt_limit_target(joint_names, weight);
      }
      else {
        add_jnt_limit_target(m_rbtree->get_movable_joint_names(), weight);
      }
    }
    else if (obj_type == "JNT_VEL") {
      double weight = obj["weight"].as<double>();
      if (obj["joint_names"].IsDefined()) {
        std::vector<std::string> joint_names
            = obj["joint_names"].as<std::vector<std::string>>();
        add_jnt_vel_target(joint_names, weight);
      }
      else {
        add_jnt_vel_target(weight);
      }
    }
    else if (obj_type == "DISTANCE") {
      std::string body_name1 = obj["body1"].as<std::string>();
      std::string body_name2 = obj["body2"].as<std::string>();
      math::Vector3d bpos1(
          obj["bpos1"][0].as<double>(),
          obj["bpos1"][1].as<double>(),
          obj["bpos1"][2].as<double>());
      math::Vector3d bpos2(
          obj["bpos2"][0].as<double>(),
          obj["bpos2"][1].as<double>(),
          obj["bpos2"][2].as<double>());
      double checking_threshold = -9999999;
      if (obj["dist"].IsDefined())
        checking_threshold = obj["dist"].as<double>();
      double weight = obj["weight"].as<double>();
      add_distance_target(
          body_name1, bpos1, body_name2, bpos2, checking_threshold, weight);
    }
    else {
      CROBOT_LOG_ERROR("Unknown objective type: {}", obj_type);
    }
  }
}

//==============================================================================
math::VectorXd NullSpaceObj::get_weighted_partial(
    math::VectorXd qpos, const math::VectorXd& qvel) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_rbtree->get_qpos_dim());

  math::VectorXd partial_deriv = math::VectorXd::Zero(qpos.size());
  for (auto obj_target : m_obj_targets) {
    partial_deriv += obj_target->get_partial_deriv(qpos, qvel);
  }

  return (partial_deriv);
}

//==============================================================================
void NullSpaceObj::add_jnt_limit_target(
    const std::vector<std::string>& joint_names, double weight) {
  auto obj_target
      = std::make_shared<ObjJntLimitTarget>(m_rbtree, joint_names, weight);
  m_obj_targets.push_back(obj_target);
}

//==============================================================================
void NullSpaceObj::add_distance_target(
    const std::string& body_name1,
    const math::Vector3d& bpos1,
    const std::string& body_name2,
    const math::Vector3d& bpos2,
    double checking_threshold,
    double weight) {

  auto obj_target = std::make_shared<ObjDistanceTarget>(
      m_rbtree,
      body_name1,
      bpos1,
      body_name2,
      bpos2,
      checking_threshold,
      weight);
  m_obj_targets.push_back(obj_target);
}
//==============================================================================
void NullSpaceObj::add_quat_target(
    const std::string& body_name,
    const math::Quaternion& quat_target,
    double weight) {

  auto obj_target = std::make_shared<ObjQuatTarget>(
      m_rbtree, body_name, quat_target, weight);
  m_obj_targets.push_back(obj_target);
}

//==============================================================================
void NullSpaceObj::add_jnt_vel_target(double weight) {
  auto obj_target = std::make_shared<ObjJntVelTarget>(
      m_rbtree, m_rbtree->get_movable_joint_names(), weight);
  m_obj_targets.push_back(obj_target);
}

//==============================================================================
void NullSpaceObj::add_jnt_vel_target(
    std::vector<std::string> joint_names, double weight) {
  auto obj_target
      = std::make_shared<ObjJntVelTarget>(m_rbtree, joint_names, weight);
  m_obj_targets.push_back(obj_target);
}
