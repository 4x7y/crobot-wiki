#pragma once

#include <set>

#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/simulation/mj_joint.hpp"
#include "crobot/simulation/mj_rigid_body.hpp"

namespace crobot::simulation {

class MJRBTree;
using MJRBTreePtr      = std::shared_ptr<MJRBTree>;
using ConstMJRBTreePtr = std::shared_ptr<const MJRBTree>;

class MJRBTree : public kdl::RigidBodyTree {
public:
  /**
   * @brief Construct a new MJRBTree object
   *
   * @param name The name of the RigidBodyTree.
   * @param model The Mujoco model.
   * @param data The Mujoco data.
   * @param root_name The name of the root body that connect to the world frame.
   * @return MJRBTreePtr A shared pointer to the MJRBTree object.
   */
  static MJRBTreePtr MakeShared(
      const std::string& name,
      const mjModel* model,
      mjData* data,
      const std::string& root_name);

  /**
   * @brief Construct a new MJRBTree including all the bodies.
   *
   * @param model The Mujoco model.
   * @param data The Mujoco data.
   * @return MJRBTreePtr
   */
  static MJRBTreePtr MakeShared(const mjModel* model, mjData* data);

  /**
   * @brief Construct a new MJRBTree object
   *
   * @param name The name of the RigidBodyTree.
   * @param model The Mujoco model.
   * @param data The Mujoco data.
   */
  MJRBTree(const std::string& name, const mjModel* model, mjData* data);

  /**
   * @brief Get robot's body handle by name.
   *
   * Returns the handler to the rigid body named @c name in the robot.
   */
  ConstMJRigidBodyPtr get_body(const std::string& name) const;

  /**
   * @brief Get robot's body handle by its joint name.
   *
   * Returns the handler to the rigid body with joint named @c name in the
   * robot.
   */
  ConstMJRigidBodyPtr get_body_by_joint(const std::string& joint_name) const;

  /**
   * @brief Get all the robot's body handles.
   *
   * Returns the handlers to the rigid bodies in the robot.
   */
  std::vector<ConstMJRigidBodyPtr> get_bodies() const;

  /**
   * @brief Get all the robot's bodies except dummy ones.
   */
  std::vector<ConstMJRigidBodyPtr> get_bodies_no_dummy() const;

  /**
   * @brief Get the transformation matrix from the body frame to the world
   * frame.
   *
   * @param body Name of the body.
   * @return math::Matrix4d The transform matrix.
   */
  math::Matrix4d get_transform_mj(const std::string& body) const;

  /**
   * @brief Get the transformation matrix from body 1 to body 2.
   *
   * @param body1 Name of the first body.
   * @param body2 Name of the second body.
   * @return math::Matrix4d The transformation matrix.
   */
  math::Matrix4d get_transform_mj(
      const std::string& body1, const std::string& body2) const;

  //==============================================================================
  //                          MUJOCO BASED APIS
  //==============================================================================

  /**
   * @brief Set the qpos of the rbtree to Mujoco data struct.
   *
   * @param qpos Robot configuration
   */
  void set_qpos_mj(const math::VectorXd& qpos) const;

  /**
   * @brief Set the qpos of a joint to Mujoco data struct.
   *
   * @param joint_name Joint name
   * @param qpos Joint configuration
   */
  void set_qpos_mj(
      const std::string& joint_name, const math::VectorXd& qpos) const;

  /**
   * @brief Set the qpos of a REVOLUTE or PRISMATIC joint to Mujoco data struct.
   *
   * @param joint_name Joint name
   * @param qpos Joint configuration
   */
  void set_qpos_mj(const std::string& joint_name, double qpos) const;

  /**
   * @brief Set the qpos of a given body' joint to Mujoco data struct.
   *
   * @param body_name Body name
   * @param qpos Joint configuration
   */
  void set_qpos_by_body_mj(
      const std::string& body_name, const math::VectorXd& qpos) const;

  /**
   * @brief Get the qpos of the rbtree from Mujoco data struct.
   *
   * @return math::VectorXd Robot configuration
   */
  math::VectorXd get_qpos_mj() const;

  /**
   * @brief Get the joint configuration by joint name.
   *
   * @param joint_name Name of the joint
   * @return math::VectorXd Joint configuration
   */
  math::VectorXd get_qpos_mj(const std::string& joint_name) const;

  /**
   * @brief Get the joint configuration by body name.
   *
   * @param body_name Name of the body
   * @return math::VectorXd Joint configuration
   */
  math::VectorXd get_qpos_by_body(const std::string& body_name) const;

  void set_qvel_mj(const math::VectorXd& qvel) const;

  void set_qvel_mj(
      const std::string& joint_name, const math::VectorXd& qvel) const;

  math::VectorXd get_qvel_mj() const;

  void set_qacc_mj(const math::VectorXd& qacc) const;

  math::VectorXd get_qacc_mj() const;

  void set_ctrl_mj(const math::VectorXd& ctrl) const;

  math::VectorXd get_ctrl_mj() const;

  void set_qfrc_applied_mj(const math::VectorXd& qfrc_applied) const;

  /**
   * @brief Get the joint actuation force from Mujoco data struct. This is the
   * result of the inverse dynamics.
   *
   * @return math::VectorXd Joint actuational force.
   */
  math::VectorXd get_qfrc_inverse_mj() const;

  /**
   * @brief Set the external wreches to a specified body of the robot.
   *
   * @param data Pointer to a Mujoco mjData struct.
   * @param wrench External wrench vector. The wrench vectors are applied to the
   * robot body from the environment and expressed in the robot base frame. Note
   * a wrench (rxf + m, f) is different to a simple stacking of torque and force
   * (m, f). You can add wrench together to specify multiple wrenches on this
   * body.
   */
  void set_external_wrench(
      const std::string& bodyname, const math::Vector6d& wrench) const;

  /**
   * @brief Set the external torque and force in the world frame to a specified
   * body of the robot.
   *
   * @param bodyname Name of the body.
   * @param xtorq External torque in the world frame.
   * @param xfrc  External force in the world frame.
   */
  void set_xtorq_xfrc(
      const std::string& bodyname,
      const math::Vector3d& xtorq,
      const math::Vector3d& xfrc) const;

  /**
   * @brief Set the external wreches to the bodies of the robot.
   *
   * @param data Pointer to a Mujoco mjData struct.
   * @param wrench External wrench vectors, specified as a @c std::vector
   * containing @c n wrench vectors, where @c n is number of bodies in the
   * robot. The wrench vectors are applied to the robot bodies and expressed in
   * the robot base frame. Note a wrench (rxf + m, f) is different to a simple
   * stacking of torque and force (m, f). You can add wrench matrices together
   * to specify multiple forces on multiple bodies.
   */
  void set_external_wrench(const std::vector<math::Vector6d>& wrench) const;

  /**
   * @brief Forward kinematics
   *
   * mj_kinematics will be called with the current qpos values. The following
   * frames, axis, and objects will be updated:
   * 1. Cartesian position of body frame      (xpos)
   * 2. Cartesian orientation of body frame   (xquat)
   * 3. Cartesian orientation of body frame   (xmat)
   * 4. Cartesian position of body com        (xipos)
   * 5. Cartesian orientation of body inertia (ximat)
   * 6. Cartesian position of joint anchor    (xanchor)
   * 7. Cartesian joint axis                  (xaxis)
   * 8. Cartesian geom position               (geom_xpos)
   * 9. Cartesian geom orientation            (geom_xmat)
   * 10. Cartesian site position              (site_xpos)
   * 11. Cartesian site orientation           (site_xmat)
   */
  void forward_kinematics_mj() const;

  /**
   * @brief Get the body jacbian.
   */
  math::Jacobian get_body_jacb_mj(const std::string& bodyname) const;

  /**
   * @brief Joint-space mass matrix given the joint configurations @c qpos .
   *
   * Armature inertia (or rotor inertia, or reflected inertia) is considered in
   * this implementation.
   */
  math::MatrixXd mass_matrix_with_armature_inertia(
      const math::VectorXd& qpos) const;

  /**
   * @brief Joint-space mass matrix at the current joint configurations.
   */
  math::MatrixXd mass_matrix_mj() const;

  /**
   * @brief Center of mass of the rigid body tree.
   */
  math::Vector3d center_of_mass_mj() const;

  /**
   * @brief Compute the joint torques that cancel velocity dependent term.
   *
   * Computes joint torques @c tau required for ROBOT to cancel the forces
   * induced by joint velocities @c qvel at the joint configuration @c qpos.
   */
  math::VectorXd velocity_product_mj() const;

  /**
   * @brief Required joint torques for given motion.
   *
   * Computes joint torques for the specified joint configuration, velocities,
   * accelerations, and external wrenches. Use the @c compose_external_wrench
   * function to generate @c wrench .
   *
   * @param qpos Joint configurations
   * @param qvel Joint velocities
   * @param qacc Joint accelerations
   * @param wrench External wrench vectors, specified as a @c std::vector
   * containing @c n wrench vectors, where @c n is number of bodies in the
   * robot. The wrench vectors are applied to the robot bodies and expressed in
   * the robot base frame. Note a wrench (rxf + m, f) is different to a simple
   * stacking of torque and force (m, f). You can add wrench matrices together
   * to specify multiple forces on multiple bodies.
   */
  math::VectorXd inverse_dynamics(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const math::VectorXd& qacc,
      const std::vector<math::Vector6d>& wrench) const override;

  using kdl::RigidBodyTree::inverse_dynamics;

  /**
   * @brief Inverse dynamics
   *
   * The inputs to inverse dynamics are the following fields of mjData:
   *
   *             (mjData.qpos, mjData.qvel, mjData.qacc).
   *
   * The main output is @p qfrc_inverse (for most of the time, tau + J^T f).
   *
   * This is the force that must have acted on the system in order to achieve
   * the observed acceleration @p qacc . If forward dynamics were to be computed
   * exactly, by running the iterative solver to full convergence, we would have
   *
   *   qfrc_inverse = qfrc_applied + Jacobian'*xfrc_applied + qfrc_actuator
   *                  ------------             ------------   -------------
   *                 (qfrc_applied)           (xfrc_applied)   (from ctrl)
   *
   * where @p qfrc_actuator is the joint-space force produced by the actuators
   * (derived from actuation model given @p ctrl ) and the @p Jacobian is the
   * mapping from joint to Cartesian space.
   *
   * If there is no external forces exerted to the manipulator ( @p xfrc_applied
   * = 0) and joint friction is negligible ( @p qfrc_applied = 0), we have
   *
   *   qfrc_inverse = qfrc_actuator    (joint_effort, i.e. tau)
   *
   * When the "fwdinv" flag in mjModel.opt.enableflags is set, the above
   * identity is used to monitor the quality of the forward dynamics solution.
   * In particular, the two components of mjData.solver_fwdinv are set to the L2
   * norm of the difference between the forward and inverse solutions, in terms
   * of joint forces and constraint forces respectively.
   */
  math::VectorXd inverse_dynamics_mj() const;

  /**
   * @brief Forward dynamics
   *
   * The top-level function mj_forward invokes the sequence of computations
   * below.
   *
   * -# Compute the forward kinematics. This yields the global positions and
   *    orientations of all bodies, geoms, sites, cameras and lights. It also
   *    normalizes all quaternions, just in case.
   * -# Compute the body inertias and joint axes, in global frames centered at
   *    the centers of mass of the corresponding kinematic subtrees (to improve
   *    floating-point accuracy).
   * -# Compute the tendon lengths and moment arms. This includes the
   *    computation of minimal-length paths for spatial tendons.
   * -# Compute the actuator lengths and moment arms.
   * -# Compute the composite rigid body inertias and construct the joint-space
   *    inertia matrix.
   * -# Compute the sparse factorization of the joint-space inertia
   *    matrix.
   * -# Construct the list of active contacts. This includes both broad-phase
   *    and near-phase collision detection.
   * -# Construct the constraint Jacobian and compute the constraint residuals.
   * -# Compute the matrices and vectors needed by the constraint solvers.
   * -# Compute sensor data that only depends on position, and the potential
   *    energy if enabled.
   * -# Compute the tendon and actuator velocities.
   * -# Compute the body velocities and rates of change of the joint axes,
   *    again in the global coordinate frames centered at the subtree centers of
   *    mass.
   * -# Compute all passive forces: spring-dampers in joints and tendons, and
   *    fluid dynamics forces.
   * -# Compute sensor data that depends on velocity, and the kinetic energy if
   *    enabled.
   * -# Compute the reference constraint acceleration.
   * -# Compute the vector of Coriolis, centrifugal and gravitational forces.
   * -# Compute the actuator forces and activation dynamics if defined.
   * -# Compute the joint acceleration resulting from all forces except for the
   *    (still unknown) constraint forces.
   * -# Compute the constraint forces with the selected solver, and update the
   *    joint acceleration so as to account for the constraint forces. This
   *    yields the vector mjData.qacc which is the main output of forward
   *    dynamics.
   * -# Compute sensor data that depends on force and acceleration if enabled.
   *
   * @return math::VectorXd Joint acceleration
   */
  math::VectorXd forward_dynamics_mj() const;

  /**
   * @brief Run collision detection.
   */
  void collision_checking_mj() const;

  void show_addrs() const;

  /**
   * @brief Get MuJoCo model
   *
   * @return const mjModel* Pointer to MuJoCo model
   */
  const mjModel* get_model_mj() const;

  /**
   * @brief Get the data mj object
   *
   * @return mjData*
   */
  mjData* get_data_mj() const;

  /**
   * @brief Check whether the RigidBodyTree is in collision.
   */
  bool check_collision() const;

  /**
   * @brief Get the contacts object
   *
   * @return std::vector<kdl::BodyContact::ConstPtr>
   */
  std::vector<kdl::BodyContact::ConstPtr> get_contacts() const;

private:
  /**
   * @brief Add a MJRigidBody instance to the robot. Update the @c m_qpos_addr ,
   * and @c m_qvel_addr .
   */
  void add_body_mj(MJRigidBodyPtr body, const std::string& parent_name);

  /**
   * @brief Add a Mujoco body to this RigidBodyTree.
   *
   * @param mjid_parent The id of the parent of the body to be added.
   * @param mjid_body The id of the body to be added.
   */
  void add_body_mj(int mjid_parent, int mjid_body);

  /**
   * @brief Add a Mujoco site to this RigidBodyTree.
   *
   * @param mjid_site The id of the site to be added.
   */
  void add_site_mj(int mjid_site);

  /**
   * @brief Add a joint tendon with givien coefficients to the RigidBodyTree.
   */
  kdl::ConstTendonPtr add_joint_tendon_mj(
      int mjid_tendon,
      const std::vector<int>& joints,
      const std::vector<double>& coeffs);

  /**
   * @brief Add a joint actuator to the RigidBodyTree.
   */
  kdl::ConstActuatorPtr add_joint_actuator_mj(
      int mjid_actuator, int mjid_joint);

  /**
   * @brief Add a tendon actuator to the RigidBodyTree.
   */
  kdl::ConstActuatorPtr add_tendon_actuator_mj(
      int mjid_actuator, int mjid_tendon);

  int body_name2id(const std::string& bodyname) const;

  math::Matrix4d get_transform_mj(int mjid_body) const;

  math::Matrix4d get_transform_mj(const int mjid_body1, int mjid_body2) const;

  math::LinearJacobian get_body_jacp_mj(int mjid_body) const;

  math::LinearJacobian get_body_jacp_at_xpos_mj(
      int bid, const math::Vector3d& xpos) const;

  math::AngularJacobian get_body_jacr_mj(int bid) const;

  math::Jacobian get_body_jacb_mj(int bid) const;

  math::Jacobian get_body_jacb_at_xpos(
      int body, const math::Vector3d& xpos) const;

  math::Matrix3X get_body_jacp_dot(int bid) const;

  math::Matrix3X get_body_jacr_dot(int bid) const;

  math::Matrix6X get_body_jacb_dot(int bid) const;

  void set_external_wrench(int bid, const math::Vector6d& wrench) const;

  void set_xtorq_xfrc(
      int bid, const math::Vector3d& xtorq, const math::Vector3d& xfrc) const;

  const mjModel* m_model;
  mjData* m_data;

  std::pair<int, int> m_qpos_addr;
  std::pair<int, int> m_qvel_addr;
  std::pair<int, int> m_ctrl_addr;
  std::set<int> m_body_addr;
  std::set<int> m_joint_addr;
  std::set<int> m_tendon_addr;
  std::set<int> m_actuator_addr;

  friend class MJRigidBody;
};

} // namespace crobot::simulation

