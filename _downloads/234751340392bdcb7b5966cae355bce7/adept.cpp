#include "crobot/hardware/adept.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include "crobot/config.hpp"
#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/gik.hpp"
#include "crobot/simulation/mj_rigid_body_tree.hpp"
#include "crobot/simulation/mj_sim.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace crobot;
using namespace hardware;

//==============================================================================
const double Adept::HARD_LIMIT::RADIO_REACH_LOWER = 0.1626;
const double Adept::HARD_LIMIT::RADIO_REACH_UPPER = 0.6000;
const double Adept::HARD_LIMIT::X_LOWER           = 0.2000;
const double Adept::HARD_LIMIT::X_UPPER           = 0.6000;
const double Adept::HARD_LIMIT::Y_LOWER           = -0.6000;
const double Adept::HARD_LIMIT::Y_UPPER           = +0.6000;
const double Adept::HARD_LIMIT::Z_LOWER           = 0.1770;
const double Adept::HARD_LIMIT::Z_UPPER           = 0.3870;
const double Adept::HARD_LIMIT::ROLL_LOWER        = -2. * M_PI;
const double Adept::HARD_LIMIT::ROLL_UPPER        = +2. * M_PI;

//==============================================================================
const double Adept::SOFT_LIMIT::RADIO_REACH_LOWER = 0.2500;
const double Adept::SOFT_LIMIT::RADIO_REACH_UPPER = 0.5900;
const double Adept::SOFT_LIMIT::X_LOWER           = 0.2500;
const double Adept::SOFT_LIMIT::X_UPPER           = 0.5800;
const double Adept::SOFT_LIMIT::Y_LOWER           = -0.5800;
const double Adept::SOFT_LIMIT::Y_UPPER           = +0.5800;
const double Adept::SOFT_LIMIT::Z_LOWER           = 0.1800;
const double Adept::SOFT_LIMIT::Z_UPPER           = 0.3850;
const double Adept::SOFT_LIMIT::ROLL_LOWER        = -2. * M_PI;
const double Adept::SOFT_LIMIT::ROLL_UPPER        = +2. * M_PI;

//==============================================================================
const double Adept::HARD_LIMIT::J1_LOWER = -1.8326;
const double Adept::HARD_LIMIT::J1_UPPER = +1.8326;
const double Adept::HARD_LIMIT::J2_LOWER = -2.6180;
const double Adept::HARD_LIMIT::J2_UPPER = +2.6180;
const double Adept::HARD_LIMIT::J3_LOWER = +0.0000;
const double Adept::HARD_LIMIT::J3_UPPER = +0.2100;
const double Adept::HARD_LIMIT::J4_LOWER = -6.2831;
const double Adept::HARD_LIMIT::J4_UPPER = +6.2831;

//==============================================================================
const int Adept::CYCLE_TIME_MS   = 16;
const double Adept::CYCLE_TIME_S = 0.016;

//==============================================================================
Adept& Adept::instance() {
  // This line only runs once, thus creating the only instance in existence
  static Adept instance = Adept();
  // dereferencing the variable here, saves the caller from having to use
  // the arrow operator, and removes temptation to try and delete the
  // returned instance.
  return instance; // always returns the same instance
}

//==============================================================================
Adept::Adept() : m_initialized(false) {
  static simulation::MJSim sim(CROBOT_RESOURCE_LOCAL_PATH "adept/adept.xml");
  m_rbtree = sim.get_rbtree("adept_link_1");
  CROBOT_ASSERT_ERROR(m_rbtree != nullptr);
  m_end_effector = m_rbtree->get_body("adept_link_4");
  CROBOT_ASSERT_ERROR(m_end_effector != nullptr);

  std::string ik_filename
      = CROBOT_RESOURCE_LOCAL_PATH "ik_problems/adept_xpos2joint.yaml";
  m_ik_config = YAML::LoadFile(ik_filename);
  CROBOT_ASSERT_ERROR(m_ik_config.IsDefined());
}

//==============================================================================
bool Adept::initialize() {
  m_io_service = std::make_unique<boost::asio::io_service>();
  m_socket     = std::make_unique<boost::asio::ip::tcp::socket>(*m_io_service);
  m_socket->connect(boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string("172.16.100.200"), 1234));
  bool ret = true;
  ret &= m_socket->is_open();
  ret &= set_speed(3);
  ret &= m_socket->is_open();
  return ret;
}

//==============================================================================
bool Adept::calibrate() {
  bool ret = true;
  ret &= send_string_and_wait("CALIBRATE");
  ret &= set_speed(3);
  return ret;
}

//==============================================================================
bool Adept::move(double x, double y, double z, double roll) {
  return move_helper("MOVE", x, y, z, roll);
}

//==============================================================================
bool Adept::move(const math::Vector4d& xpos) {
  return move(xpos.x(), xpos.y(), xpos.z(), xpos.w());
}

//==============================================================================
bool Adept::move_straight(double x, double y, double z, double roll) {
  return move_helper("MOVES", x, y, z, roll);
}

//==============================================================================
bool Adept::move_straight(const math::Vector4d& xpos) {
  return move_straight(xpos.x(), xpos.y(), xpos.z(), xpos.w());
}

//==============================================================================
bool Adept::move_and_wait(double x, double y, double z, double roll) {
  return move_helper("MOVE_BREAK", x, y, z, roll);
}

//==============================================================================
bool Adept::move_and_wait(const math::Vector4d& xpos) {
  return move_and_wait(xpos.x(), xpos.y(), xpos.z(), xpos.w());
}

//==============================================================================
bool Adept::move_helper(
    const std::string& move_command,
    double x,
    double y,
    double z,
    double roll) {
  bool ret = true;
  ret &= send_string(move_command);
  ret &= wait_for_ready();

  std::ostringstream sstream;
  sstream << std::fixed;
  sstream << std::setprecision(2);
  sstream << x * 1000.0;
  sstream << ", ";
  sstream << y * 1000.0;
  sstream << ", ";
  sstream << z * 1000.0;
  sstream << ", ";
  sstream << roll / M_PI * 180.0;
  std::string msg = sstream.str();
  ret &= send_string(msg);

  ret &= wait_for_final_response();
  return ret;
}

//==============================================================================
bool Adept::enable_alter_mode() {
  return send_string_and_wait("ALTON");
}

//==============================================================================
bool Adept::disable_alter_mode() {
  return send_string_and_wait("ALTOFF");
}

//==============================================================================
bool Adept::alter(double dx, double dy, double dz, double dr) {
  std::ostringstream sstream;
  sstream << std::fixed;
  sstream << std::setprecision(2);
  sstream << dx * 1000.0;
  sstream << ", ";
  sstream << dy * 1000.0;
  sstream << ", ";
  sstream << dz * 1000.0;
  sstream << ", ";
  sstream << dr / M_PI * 180.0;
  std::string msg = sstream.str();

  bool ret = true;
  ret &= send_string(msg);
  return ret;
}

//==============================================================================
bool Adept::get_position(math::Vector4d& pos) {
  bool ret = send_string_and_wait("POSITION");
  pos      = m_last_pos;
  return ret;
}

//==============================================================================
math::Vector4d Adept::map_qpos2xpos(const math::Vector4d& qpos) const {
  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  math::Vector3d xpos_end_effector   = m_end_effector->get_xpos(tforms);

  math::Vector4d xpos;
  xpos <<                   //
      xpos_end_effector(0), //
      xpos_end_effector(1), //
      xpos_end_effector(2), //
      math::wrap_to_pi(qpos[0] - qpos[1] - qpos[3]);
  return xpos;
}

//==============================================================================
math::Vector4d Adept::map_xpos2qpos(const math::Vector4d& xpos) {
  double x = xpos[0];
  double y = xpos[1];
  double z = xpos[2];
  double w = xpos[3];

  // update visualization
  m_ik_config["problem"]["constraint"][0]["xpos"]["x"] = x;
  m_ik_config["problem"]["constraint"][0]["xpos"]["y"] = y;
  m_ik_config["problem"]["constraint"][0]["xpos"]["z"] = z;

  std::shared_ptr<ik::GeneralizedIK> ikprob
      = ik::GeneralizedIK::LoadFromYamlNode(m_rbtree, m_ik_config);
  bool ik_solved = ikprob->solve(100, 1.0);

  CROBOT_ASSERT_ERROR(
      ik_solved, "Failed to solve IK: xpos = [%f, %f, %f, %f]", x, y, z, w);

  math::VectorXd qpos = ikprob->get_qpos();
  math::Vector4d qpos_adept;
  qpos_adept << //
      qpos[0],  //
      qpos[1],  //
      qpos[2],  //
      math::wrap_to_pi(qpos[0] - qpos[1] - w);
  return qpos_adept;
}

//==============================================================================
bool Adept::set_speed(double speed_factor) {
  CROBOT_ASSERT_ERROR(
      0 <= speed_factor && speed_factor <= 100,
      "Adept speed factor out of range 0-100.");

  bool ret = true;
  ret &= send_string("SPEED");
  ret &= wait_for_ready();
  ret &= send_float(speed_factor);
  ret &= wait_for_final_response();
  return ret;
}

//==============================================================================
bool Adept::send_float(double x) const {
  std::ostringstream sstream;
  sstream << std::fixed;
  sstream << std::setprecision(2);
  sstream << x;
  std::string msg = sstream.str();
  return send_string(msg);
}

//==============================================================================
bool Adept::send_integer(int x) const {
  return send_string(std::to_string(x));
}

//==============================================================================
bool Adept::send_string_and_wait(const std::string& msg) {
  bool ret = true;
  ret &= send_string(msg);
  ret &= wait_for_final_response();
  return ret;
}

//==============================================================================
bool Adept::send_string(const std::string& msg) const {
  if (!m_socket->is_open()) {
    CROBOT_LOG_ERROR(
        "Failed to send command \"{}\" to the Adept robot. Socket is not open.",
        msg);
    return false;
  }

  boost::system::error_code error;
  boost::asio::write(
      *m_socket, boost::asio::buffer(msg.data(), msg.size()), error);
  return error == boost::system::errc::success;
}

//==============================================================================
bool Adept::wait_for_ready() const {
  boost::system::error_code error;
  boost::asio::streambuf receive_buffer;
  boost::asio::read_until(*m_socket, receive_buffer, "READY");

  if (error && error != boost::asio::error::eof) {
    CROBOT_LOG_ERROR(
        "Failed to receive response from the Adept robot. Error: %s",
        error.message().c_str());
    return false;
  }
  return true;
}

//==============================================================================
bool Adept::wait_for_final_response() {
  std::string message;
  if (!wait_until("READY", message)) {
    return false;
  }

  std::stringstream ss(message);
  double x, y, z, roll;
  ss >> x;
  ss >> y;
  ss >> z;
  ss >> roll;

  // convert mm to m
  x /= 1000.0;
  y /= 1000.0;
  z /= 1000.0;
  // convert deg to rad
  roll /= (180.0 / M_PI);

  CROBOT_ASSERT_ERROR(-0.3 <= x && x <= 0.6, "x out of range");
  CROBOT_ASSERT_ERROR(-0.6 <= y && y <= 0.6, "y out of range");
  CROBOT_ASSERT_ERROR(0.1770 <= z && z <= 0.3870, "z out of range");
  CROBOT_ASSERT_ERROR(
      -2.0 * M_PI <= roll && roll <= 2.0 * M_PI, "roll out of range");

  m_last_pos = math::Vector4d(x, y, z, roll);
  return true;
}

//==============================================================================
bool Adept::wait_until(const std::string& delim, std::string& msg) const {
  boost::system::error_code error;
  boost::asio::streambuf receive_buffer;
  boost::asio::read_until(*m_socket, receive_buffer, delim);

  if (error && error != boost::asio::error::eof) {
    CROBOT_LOG_ERROR(
        "Failed to receive response from the Adept robot. Error: %s",
        error.message().c_str());
    return false;
  }

  std::string message_received(
      boost::asio::buffers_begin(receive_buffer.data()),
      boost::asio::buffers_end(receive_buffer.data()));
  msg = message_received;
  return true;
}
