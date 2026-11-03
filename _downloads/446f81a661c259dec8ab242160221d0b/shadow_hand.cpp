#include "crobot/hardware/shadow_hand.hpp"

#include <iomanip>

#include <experimental/filesystem>
#include <yaml-cpp/yaml.h>

#include "crobot/common/tabulate.hpp"
#include "crobot/hardware/shadow_config.hpp"
#include "crobot/hardware/shadow_finger.hpp"
#include "crobot/hardware/shadow_motor.hpp"
#include "crobot/math/math.hpp"
#include "crobot/config.hpp"
#include "crobot/simulation/mj_sim.hpp"

using namespace crobot;
using namespace hardware;

static const std::string FILENAME_SHADOW_HAND_MUJOCO_MODEL
    = CROBOT_RESOURCE_LOCAL_PATH "shadow_description/sr_hand_btsp_env.xml";

static const std::string FILENAME_SHADOW_HAND_ACTUATOR_FRICTION_DATA
    = CROBOT_RESOURCE_LOCAL_PATH "shadow_calibration/actuator_friction_data/";

static simulation::MJSim sim(FILENAME_SHADOW_HAND_MUJOCO_MODEL);

std::vector<std::string> ShadowHand::ROS_JOINT_NAMES
    = {"rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_LFJ1", "rh_LFJ2",
       "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_MFJ1", "rh_MFJ2", "rh_MFJ3",
       "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", "rh_THJ1",
       "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5", "rh_WRJ1", "rh_WRJ2"};

std::vector<double> ShadowHand::ROS_JOINT_UPPER_LIMITS(24);
std::vector<double> ShadowHand::ROS_JOINT_LOWER_LIMITS(24);

//==============================================================================
ShadowHand::Ptr ShadowHand::MakeShared() {
  return MakeShared(sim.get_rbtree("rh_forearm"));
}

//==============================================================================
ShadowHand::Ptr ShadowHand::MakeShared(kdl::ConstRigidBodyTreePtr model) {

  ShadowHand::Ptr hand = std::shared_ptr<ShadowHand>(new ShadowHand(model));

  ROS_JOINT_UPPER_LIMITS.clear();
  ROS_JOINT_LOWER_LIMITS.clear();

  math::VectorXd qpos_min, qpos_max;
  std::tie(qpos_min, qpos_max) = model->get_qpos_limits();
  std::vector<int> qpos_addrs  = model->get_qpos_addrs(ROS_JOINT_NAMES);

  for (std::size_t i = 0; i < ROS_JOINT_NAMES.size(); i++) {
    int qpos_addr = qpos_addrs[i];
    ROS_JOINT_UPPER_LIMITS.push_back(qpos_max(qpos_addr));
    ROS_JOINT_LOWER_LIMITS.push_back(qpos_min(qpos_addr));
  }

  std::shared_ptr<ShadowConfig> config
      = std::make_shared<ShadowConfig>(hand->m_hand);

  hand->m_fingers.insert({"rh_ff", config->get_finger("rh_ff")});
  hand->m_fingers.insert({"rh_mf", config->get_finger("rh_mf")});
  hand->m_fingers.insert({"rh_rf", config->get_finger("rh_rf")});
  hand->m_fingers.insert({"rh_lf", config->get_finger("rh_lf")});
  hand->m_fingers.insert({"rh_th", config->get_finger("rh_th")});

  return hand;
}

//==============================================================================
ShadowHand::ShadowHand(kdl::ConstRigidBodyTreePtr robot) : m_hand(robot) {
}

//==============================================================================
kdl::ConstRigidBodyTreePtr ShadowHand::get_model() const {
  return m_hand;
}

//==============================================================================
std::shared_ptr<const ShadowFinger> ShadowHand::get_finger(
    const std::string& finger_name) const {
  return m_fingers.find(finger_name)->second;
}

//==============================================================================
std::vector<std::string> ShadowHand::get_joint_names() const {
  return m_hand->get_movable_joint_names();
}

//==============================================================================
void ShadowHand::print_details() const {
  tabulate::Table table;
  table.add_row({"name", "Shadow Hand"});

  for (const auto& pair : m_fingers) {
    std::string finger_name  = pair.first;
    ShadowFinger::Ptr finger = pair.second;

    tabulate::Table finger_info;
    finger_info
        .add_row({"motor_names: ",
                  std::accumulate(
                      finger->m_motors.begin(),
                      finger->m_motors.end(),
                      std::string{},
                      [&](const std::string& a, const ShadowMotor::Ptr& b) {
                        return a + " " + b->get_name();
                      })})
        .format()
        .hide_border();

    finger_info
        .add_row({"motor_ids: ",
                  std::accumulate(
                      finger->m_motors.begin(),
                      finger->m_motors.end(),
                      std::string{},
                      [&](const std::string& a, const ShadowMotor::Ptr& b) {
                        return a + " " + std::to_string(b->get_id());
                      })})
        .format()
        .hide_border();

    finger_info
        .add_row({"joint_names: ",
                  std::accumulate(
                      finger->m_joints.begin(),
                      finger->m_joints.end(),
                      std::string{},
                      [&](const std::string& a, const ShadowJoint::Ptr& b) {
                        return a + " " + b->get_name();
                      })})
        .format()
        .hide_border();

    finger_info
        .add_row({"qpos_addr: ",
                  std::accumulate(
                      finger->m_joints.begin(),
                      finger->m_joints.end(),
                      std::string{},
                      [&](const std::string& a, const ShadowJoint::Ptr& b) {
                        return a + " " + std::to_string(b->get_qpos_addr());
                      })})
        .format()
        .hide_border();

    finger_info
        .add_row({"k_s_pos: ",
                  std::accumulate(
                      finger->m_motors.begin(),
                      finger->m_motors.end(),
                      std::string{},
                      [&](const std::string& a, const ShadowMotor::Ptr& b) {
                        std::stringstream ss;
                        ss << "[";
                        for (std::size_t i = 0;
                             i < b->m_strain_gauge.factors.size();
                             i++) {
                          double value = b->m_strain_gauge.factors[i].k_s_pos;
                          if (std::abs(value) < 1e-8) {
                            ss << std::setw(10) << "0";
                          }
                          else {
                            ss << std::fixed << std::setprecision(6)
                               << std::setw(10) << value;
                          }
                        }
                        ss << "]";
                        return a + " " + ss.str();
                      })})
        .format()
        .hide_border();

    finger_info
        .add_row({"k_s_neg: ",
                  std::accumulate(
                      finger->m_motors.begin(),
                      finger->m_motors.end(),
                      std::string{},
                      [&](const std::string& a, const ShadowMotor::Ptr& b) {
                        std::stringstream ss;
                        ss << "[";
                        for (std::size_t i = 0;
                             i < b->m_strain_gauge.factors.size();
                             i++) {
                          double value = b->m_strain_gauge.factors[i].k_s_neg;
                          if (std::abs(value) < 1e-8) {
                            ss << std::setw(10) << "0";
                          }
                          else {
                            ss << std::fixed << std::setprecision(6)
                               << std::setw(10) << value;
                          }
                        }
                        ss << "]";
                        return a + " " + ss.str();
                      })})
        .format()
        .hide_border();

    table.add_row({"finger: " + finger->m_name, finger_info});
  }

  std::cout << table << std::endl;
}

//==============================================================================
std::vector<double> ShadowHand::get_joint_upper_limits_ros() const {
  return ShadowHand::ROS_JOINT_UPPER_LIMITS;
}

//==============================================================================
std::vector<double> ShadowHand::get_joint_lower_limits_ros() const {
  return ShadowHand::ROS_JOINT_LOWER_LIMITS;
}
