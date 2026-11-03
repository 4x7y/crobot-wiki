
#pragma once

#include <memory>

#include <boost/asio.hpp>
#include <yaml-cpp/yaml.h>

#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot {
namespace hardware {

class Adept {
public:
  struct HARD_LIMIT
  {
    static const double RADIO_REACH_LOWER;
    static const double RADIO_REACH_UPPER;
    static const double X_LOWER;
    static const double X_UPPER;
    static const double Y_LOWER;
    static const double Y_UPPER;
    static const double Z_LOWER;
    static const double Z_UPPER;
    static const double ROLL_LOWER;
    static const double ROLL_UPPER;

    static const double J1_LOWER;
    static const double J1_UPPER;
    static const double J2_LOWER;
    static const double J2_UPPER;
    static const double J3_LOWER;
    static const double J3_UPPER;
    static const double J4_LOWER;
    static const double J4_UPPER;
  };

  struct SOFT_LIMIT
  {
    static const double RADIO_REACH_LOWER;
    static const double RADIO_REACH_UPPER;
    static const double X_LOWER;
    static const double X_UPPER;
    static const double Y_LOWER;
    static const double Y_UPPER;
    static const double Z_LOWER;
    static const double Z_UPPER;
    static const double ROLL_LOWER;
    static const double ROLL_UPPER;
  };

  static const int CYCLE_TIME_MS;
  static const double CYCLE_TIME_S;

public:
  /**
   * @brief Singleton instance of the Adept robot class.
   *
   * @return Adept& The singleton instance of the Adept robot class.
   */
  static Adept& instance();

  /**
   * @brief Initializes the Adept robot.
   *
   * @return bool True if the robot was successfully initialized.
   */
  bool initialize();

  /**
   * @brief Calibrate the Adept robot.
   */
  bool calibrate();

  /**
   * @brief Moves the Adept robot to the specified world frame coordinates. The
   * trajectory is interpolated in the robot's configuration space.
   *
   * The BREAK instruction suspends program execution until the current motion
   * completes. It can be executed by any program task so long as the robot
   * selected by the task is not attached by any other task. The instruction
   * applies to the robot selected by the task. See V+ language reference page
   * 100.
   *
   * @param x The x coordinate in world frame in unit m.
   * @param y The y coordinate in world frame in unit m.
   * @param z The z coordinate in world frame in unit m.
   * @param roll The roll angle in world frame in unit radian.
   * @return bool True if the robot was successfully moved. False otherwise.
   */
  bool move(double x, double y, double z, double roll);

  /**
   * @brief Moves the Adept robot to the specified world frame coordinates. The
   * trajectory is interpolated in the robot's configuration space.
   *
   * @param xpos
   * @return true
   * @return false
   */
  bool move(const math::Vector4d& xpos);

  /**
   * @brief
   *
   * @param x
   * @param y
   * @param z
   * @param roll
   * @return true
   * @return false
   */
  bool move_straight(double x, double y, double z, double roll);

  /**
   * @brief Moves the Adept robot to the specified world frame coordinates. The
   * trajectory is interpolated as straight line in the workspace.
   *
   * @param xpos
   * @return true
   * @return false
   */
  bool move_straight(const math::Vector4d& xpos);

  /**
   * @brief
   *
   * @param x
   * @param y
   * @param z
   * @param roll
   * @return true
   * @return false
   */
  bool move_and_wait(double x, double y, double z, double roll);

  /**
   * @brief
   *
   * @param xpos
   * @return true
   * @return false
   */
  bool move_and_wait(const math::Vector4d& xpos);

  /**
   * @brief Specify the speed of all subsequent robot motions commanded by a
   * robot control program.
   *
   * @param speed The speed of all subsequent robot motions in unit m/s.
   * @return true
   * @return false
   */
  bool set_speed(double speed_factor);

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool enable_alter_mode();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool disable_alter_mode();

  /**
   * @brief
   *
   * @param dx
   * @param dy
   * @param dz
   * @param dr
   * @return true
   * @return false
   */
  bool alter(double dx, double dy, double dz, double dr);

  bool get_position(math::Vector4d& position);

  /**
   * @brief
   *
   * @param qpos
   * @return math::Vector4d
   */
  math::Vector4d map_qpos2xpos(const math::Vector4d& qpos) const;

  /**
   * @brief
   *
   * @param xpos
   * @return math::Vector4d
   */
  math::Vector4d map_xpos2qpos(const math::Vector4d& xpos);

private:
  // We need to make some given functions private to finish the definition of
  // the singleton default constructor available only to members or friends of
  // this class
  Adept();

  // Note that the next two functions are not given bodies, thus any attempt
  // to call them implicitly will return as compiler errors. This prevents
  // accidental copying of the only instance of the class.
  // Disallow copy constructor
  Adept(const Adept& old);
  // Disallow assignment operator
  const Adept& operator=(const Adept& old);

  // Note that although this should be allowed,
  // some compilers may not implement private destructors
  // This prevents others from deleting our one single instance,
  // which was otherwise created on the heap
  ~Adept() = default;

  /**
   * @brief Send
   *
   * @param x
   * @return true
   * @return false
   */
  bool send_float(double x) const;

  /**
   * @brief
   *
   * @param x
   * @return true
   * @return false
   */
  bool send_integer(int x) const;

  /**
   * @brief Send a string command to the robot.
   *
   * @param msg The message to send.
   * @return true if the message was sent successfully.
   * @return false If the message could not be sent.
   */
  bool send_string(const std::string& msg) const;

  /**
   * @brief
   *
   * @param msg
   * @return true
   * @return false
   */
  bool send_string_and_wait(const std::string& msg);

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool wait_for_final_response();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool wait_for_ready() const;

  /**
   * @brief
   *
   * @param delim
   * @return true
   * @return false
   */
  bool wait_until(const std::string& delim, std::string& msg) const;

  bool move_helper(
      const std::string& move_command,
      double x,
      double y,
      double z,
      double roll);

  /// Is the robot initialized?
  bool m_initialized;

  /// IO Service
  std::unique_ptr<boost::asio::io_service> m_io_service;

  /// Socket
  std::unique_ptr<boost::asio::ip::tcp::socket> m_socket;

  /// RigidBodyTree
  kdl::ConstRigidBodyTreePtr m_rbtree;

  /// End effector
  kdl::ConstRigidBodyPtr m_end_effector;

  /// Robot IK configuration
  YAML::Node m_ik_config;

public:
  math::Vector4d m_last_pos;
};
} // namespace hardware
} // namespace crobot

