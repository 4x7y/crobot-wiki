#pragma once

#include <string>
#include <vector>

#include "crobot/kdl/actuator.hpp"
#include "crobot/kdl/coupling.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/tendon.hpp"

namespace crobot::kdl {

class RigidBodyTree;
using RigidBodyTreePtr      = std::shared_ptr<RigidBodyTree>;
using ConstRigidBodyTreePtr = std::shared_ptr<const RigidBodyTree>;

class RigidBodyTree : public std::enable_shared_from_this<RigidBodyTree> {
public:
  static RigidBodyTreePtr MakeShared(const std::string& name);

  virtual ~RigidBodyTree() = default;

  RigidBodyTree(const RigidBodyTree&) = delete;

  /**
   * @brief Add a body to the robot.
   */
  virtual void add_body(kdl::RigidBodyPtr body, const std::string& parent_name);

  /**
   * @brief Get robot's body handle by name.
   *
   * Returns the handler to the rigid body named @c name in the robot.
   */
  kdl::ConstRigidBodyPtr get_body(const std::string& name) const;

  /**
   * @brief Get robot's body handle by its joint name.
   *
   * Returns the handler to the rigid body with joint named @c name in the
   * robot.
   */
  kdl::ConstRigidBodyPtr get_body_by_joint(const std::string& joint_name) const;

  /**
   * @brief Get all the robot's body handles.
   *
   * Returns the handlers to the rigid bodies in the robot.
   */
  std::vector<ConstRigidBodyPtr> get_bodies() const;

  /**
   * @brief Get robot joint by name.
   *
   * @param name The name of the joint.
   * @return kdl::ConstJointPtr The joint handle.
   */
  kdl::ConstJointPtr get_joint(const std::string& name) const;

  /**
   * @brief Get a subtree from robot as a new robot
   *
   * @param subtree_name Name of the subtree to be extracted.
   * @param bodyname Body name of the root of the subtree.
   * @return kdl::RigidBodyTreePtr Pointer to the subtree.
   */
  kdl::RigidBodyTreePtr get_subtree(
      const std::string& subtree_name, const std::string& bodyname) const;

  /**
   * @brief Get the number of rigid bodies including the base.
   */
  virtual int get_num_bodies() const;

  /**
   * @brief Get the number of position variables.
   */
  virtual int get_qpos_dim() const;

  /**
   * @brief Get the qpos addr of the given joint
   *
   * @param joint_name Name of the joint.
   * @return std::pair<int, int> The qpos addr of the given joint.
   */
  virtual std::pair<int, int> get_qpos_addr(
      const std::string& joint_name) const;

  /**
   * @brief Get the qpos addrs of the given joints.
   *
   * @param joint_names The names of the joints.
   * @return std::vector<int> The qpos addrs of the given joint names.
   */
  virtual std::vector<int> get_qpos_addrs(
      const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the number of velocity/acceleration variables.
   */
  virtual int get_qvel_dim() const;

  /**
   * @brief Get the qvel addr of the given joint
   *
   * @param joint_name Name of the joint.
   * @return std::pair<int, int> The qvel addr of the given joint.
   */
  virtual std::pair<int, int> get_qvel_addr(
      const std::string& joint_name) const;

  /**
   * @brief Get the qvel addrs of the given joints.
   *
   * @param joint_names The names of the joints.
   * @return std::vector<int> The qvel addrs of the given joint names.
   */
  virtual std::vector<int> get_qvel_addrs(
      const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the number of control/actuator variables.
   */
  virtual int get_ctrl_dim() const;

  /**
   * @brief Subtract one joint configuration vector from the other and return
   * the difference in velocity dimension.
   */
  virtual math::VectorXd subtract_qpos(
      math::VectorXd& qpos_a, math::VectorXd& qpos_b) const;

  /**
   * @brief Add a joint coupling constraint to the RigidBodyTree.
   */
  virtual ConstCouplingPtr add_joint_coupling(
      const std::string& name,
      const std::string& joint1,
      const std::string& joint2,
      const math::VectorXd& poly);

  /**
   * @brief Add a joint tendon with givien coefficients to the RigidBodyTree.
   */
  virtual ConstTendonPtr add_joint_tendon(
      const std::string& name,
      const std::vector<std::string>& joints,
      std::vector<double> coeffs);

  /**
   * @brief Add a joint actuator to the RigidBodyTree.
   */
  virtual ConstActuatorPtr add_joint_actuator(
      const std::string& name, const std::string& joint);

  /**
   * @brief Add a tendon actuator to the RigidBodyTree.
   */
  virtual ConstActuatorPtr add_tendon_actuator(
      const std::string& name, const std::string& tendon);

  /**
   * @brief Get the matrix that describes the coupling behavior of the joint
   * velocities. The definition of the coupling matrix @c C is given as
   * C x qvel = 0.
   */
  virtual math::MatrixXd get_tmat_qvel_coupling(
      const math::VectorXd& qpos) const;

  /**
   * @brief Get the matrix that maps actuator velocities to joint velocities.
   */
  virtual math::MatrixXd get_tmat_uvel2qvel(const math::VectorXd& qpos) const;

  /**
   * @brief Get the matrix that maps joint velocities to actuator velocities.
   */
  virtual math::MatrixXd get_tmat_qvel2uvel(const math::VectorXd& qpos) const;

  /**
   * @brief Forward kinematics.
   */
  virtual std::vector<math::Matrix4d> forward_kinematics(
      const math::VectorXd& qpos) const;

  /**
   * @brief Get the transform T that converts points originally expressed in
   * 'body1' frame to 'body2' frame.
   */
  virtual math::Matrix4d get_transform(
      const math::VectorXd& qpos,
      const std::string& body1,
      const std::string& body2) const;

  /**
   * @brief Get the transform T that converts points originally expressed in
   * 'body' frame to the base frame.
   */
  virtual math::Matrix4d get_transform(
      const math::VectorXd& qpos, const std::string& body) const;

  /**
   * @brief Compute the geometric Jacobian.
   */
  virtual math::Jacobian get_body_jacb(
      const math::VectorXd& qpos,
      const std::string& body,
      JacobianType type = JacobianType::WITH_RESPECT_TO_QVEL) const;

  /**
   * @brief Compute the geometric Jacobian.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param name The body name.
   */
  math::Jacobian get_body_jacb(
      const std::vector<math::Matrix4d>& tforms, const std::string& name) const;

  math::Vector3d get_gravity_coeff() const;

  /**
   * @brief Generate random configuration of robot.
   *
   * Returns a random configuration of the specified robot. Each joint
   * position in this configuration respects the joint limits set by the
   * PositionLimits property of the corresponding rigidBodyJoint object in the
   * robot model.
   */
  virtual math::VectorXd make_random_qpos() const;

  virtual math::VectorXd make_random_qpos_offset(
      const math::VectorXd& qpos_ref,
      const math::VectorXd& qpos_offset_limit) const;

  /**
   * @brief Get home configuration of robot.
   *
   * Returns the home configuration of the robot model. The home configuration
   * is the ordered list of HomePosition properties of each nonfixed joint.
   */
  virtual math::VectorXd get_home_qpos() const;

  /**
   * @brief Generate a zero joint configuration vector of robot.
   */
  virtual math::VectorXd make_zero_qpos() const;

  /**
   * @brief Get the joint configuration limits.
   */
  virtual std::pair<math::VectorXd, math::VectorXd> get_qpos_limits() const;

  /**
   * @brief Generate a zero joint velocity vector of robot.
   */
  virtual math::VectorXd make_zero_qvel() const;

  /**
   * @brief Generate a zero joint accelerations vector of robot.
   */
  virtual math::VectorXd make_zero_qacc() const;

  /**
   * @brief Generate random joint velocities of robot.
   *
   * Returns a random joint velocities of the specified robot. Each joint
   * velocity ranges from -1 to 1.
   */
  virtual math::VectorXd make_random_qvel() const;

  /**
   * @brief Generate random joint torques of robot.
   *
   * Returns a random joint torques of the specified robot. Each joint
   * velocity ranges from -1 to 1.
   */
  virtual math::VectorXd make_random_qfrc() const;

  /**
   * @brief Get the movable joint names.
   *
   * @return std::vector<std::string> The movable joint names.
   */
  std::vector<std::string> get_movable_joint_names() const;

  /**
   * @brief Display details of robot.
   *
   * Displays details of each body in the robot including the body name,
   * associated joint name and type, and its parent name and children names.
   */
  virtual void show_details() const;

  /**
   * @brief Get the position of center of mass (COM).
   *
   * Computes the center of mass position of the robot model at the specified
   * joint configuration, relative to the base frame.
   */
  virtual math::Vector3d get_com_info(
      const math::VectorXd& qpos, math::Jacobian& comJac) const;

  /**
   * @brief Compose external force matrix relative to base
   *
   * Composes the external force matrix assuming that wrench is in the bodyname
   * frame for the specified configuration. The force matrix fext is given in
   * the base frame.
   */
  virtual std::vector<math::Vector6d> compose_external_force(
      const std::string& body,
      const math::VectorXd& qpos,
      const math::Vector6d& wrench) const;

  /**
   * @brief Compute resultant joint acceleration given joint torques and states.
   *
   * Computes the joint accelerations @c qaccc due to gravity, joint velocities
   * @c qvel , torques applied to the joints @c tau , and external forces
   * applied to each body @c fext , when ROBOT is at configuration @c qpos.
   *
   * @param qpos Joint configuration
   * @param qvel Joint velocities
   * @param tau Joint torques/forces
   * @param fext External forces
   * @return Joint accelerations
   */
  virtual math::VectorXd forward_dynamics(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const math::VectorXd& tau,
      const std::vector<math::Vector6d>& fext) const;

  /**
   * @brief Joint torques that compensate gravity.
   *
   * Computes GTORQ, the joint torques required to hold ROBOT at configuration
   * @c qpos .
   */
  virtual math::VectorXd gravity_torque(const math::VectorXd& qpos) const;

  /**
   * @brief Inverse dynamics of the RigidBodyTree. Compute required joint
   * torques for given motion.
   *
   * Given the current (or desired) joint positions and velocities, the desired
   * joint accelerations, compute the required joint torque.
   */
  virtual math::VectorXd inverse_dynamics(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const math::VectorXd& qacc) const;

  /**
   * @brief Inverse dynamics of the RigidBodyTree. Compute required joint
   * torques for given motion.
   *
   * Given the current (or desired) joint positions and velocities, the desired
   * joint accelerations, and the external forces applied to each body, compute
   * the required joint torque.
   */
  virtual math::VectorXd inverse_dynamics(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      const math::VectorXd& qacc,
      const std::vector<math::Vector6d>& fext) const;

  /**
   * @brief Joint-space mass matrix.
   *
   * Compute the mass matrix, @c H , of the robot in the configuration @c qpos .
   */
  virtual math::MatrixXd mass_matrix(const math::VectorXd& qpos) const;

  /**
   * @brief Compute the joint torques that cancel velocity dependent term.
   *
   * Computes joint torques @c tau required for ROBOT to cancel the forces
   * induced by joint velocities @c qvel at the joint configuration @c qpos.
   */
  virtual math::VectorXd velocity_product(
      const math::VectorXd& qpos, const math::VectorXd& qvel) const;

  /**
   * @brief Compute the center of mass position.
   *
   * Computes the center of mass position of ROBOT at configuration @c qpos
   * relative to the base frame.
   *
   * @param[in] qpos Joint configuration of the ROBOT.
   * @return Center of mass position of ROBOT
   */
  virtual math::VectorXd center_of_mass(const math::VectorXd& qpos) const;

  /**
   * @brief Compute the center of mass position and Jacobian
   *
   * center of mass Jacobian COMJAC as the second output argument.
   *
   * @param[in] qpos Joint configuration of the ROBOT.
   * @param[out] jacp A 3-by-vNum linear Jacobian matrix, where vNum is the
   velocity number of ROBOT (i.e. the degrees of freedom).
   * @return Center of mass position of ROBOT
   */
  virtual math::VectorXd center_of_mass(
      const math::VectorXd& qpos, math::LinearJacobian& jacp) const;

protected:
  RigidBodyTree(const std::string& name);

  /**
   * @brief Get robot's body handle by id.
   *
   * Returns the handler to the rigid body indexed @c bid in the robot.
   */
  virtual kdl::RigidBodyPtr get_body(int bid) const;

  /**
   * @brief Returns the index of the body with the provided @c name .
   *
   * If the given body name is the base name, return 0. In all other cases,
   * return -1.
   */
  int find_body_index_by_name(const std::string& name) const;

  /**
   * @brief Get the support qpos addr object.
   */
  std::vector<int> get_support_qpos_addr(int body) const;

  /**
   * @brief Get the support qvel addr object.
   */
  std::vector<int> get_support_qvel_addr(int body) const;

  /**
   * @brief Get the support bodies that associated with movable joints.
   */
  std::vector<ConstRigidBodyPtr> get_support_movable_bodies(int body) const;

  /**
   * @brief Returns the index of the body with the provided @c name if the body
   * exists. Otherwise throws error exception.
   */
  int validate_input_body_name(const std::string& name) const;

  /**
   * @brief Returns the index of the body whose joint has the name 'jointname'.
   * Only search within Bodies.
   */
  int find_body_index_by_joint_name(const std::string& name) const;

  /**
   * @brief Get the transform T that converts points originally expressed in
   * 'body1' frame to 'body2' frame.
   */
  math::Matrix4d get_transform(
      const math::VectorXd& qpos, int body1, int body2) const;

  /**
   * @brief Get the transform T that converts points originally expressed in
   * 'body' frame to the base frame.
   */
  math::Matrix4d get_transform(const math::VectorXd& qpos, int body) const;

  /**
   * @brief Get the transform T that converts points originally expressed in
   * 'body' frame to the base frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   */
  math::Matrix4d get_transform(
      const std::vector<math::Matrix4d>& tforms, int body) const;

  /**
   * @brief Compute the geometric Jacobian.
   *
   * @param qpos
   * @param body The body index.
   * @param type The type of the Jacobian.
   */
  math::Jacobian get_body_jacb(
      const math::VectorXd& qpos,
      int body,
      JacobianType type = JacobianType::WITH_RESPECT_TO_QVEL) const;

  /**
   * @brief Compute the geometric Jacobian.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param body The body index.
   */
  math::Jacobian get_body_jacb(
      const std::vector<math::Matrix4d>& tforms, int body) const;

  /**
   * @brief Compute the geometric Jacobian.
   *
   * @param qpos
   * @param body The body index.
   * @param type The type of the Jacobian.
   */
  math::Jacobian get_body_jacb(
      const math::VectorXd& qpos,
      const std::vector<math::Matrix4d>& tforms,
      int body,
      JacobianType type) const;

  /**
   * @brief Compute the composite rigid body nertia (CRBI) of each body of robot
   * under configuration @c qpos .
   *
   * @param qpos Joint configuration.
   * @param IcrbBodies An nb-size vector, where nb is the number of bodies. Each
   * cell contains the composite rigid body inertia of body i, relative to body
   * i frame.
   * @param X An nb-size vector, with entry i containing the spatial transform
   * (for motion vectors) from body parent(i) to body i.
   */
  void composite_rigid_body_inertia(
      const math::VectorXd& qpos,
      std::vector<math::Matrix6d>& IcrbBodies,
      std::vector<math::Matrix6d>& X) const;

  /**
   * @brief Joint-space mass matrix.
   *
   * Compute the mass matrix, H, of the robot in the configuration q.
   *
   * @param qpos A valid configuration (position) vector for robot.
   * @param lambda A vector of indices, lambda(i) is the column number of the
   * last nonzero entry (count from left) on row i of the lower triangle of H
   * (not counting main diagonal). If no nonzero entry is found, lambda(i) is
   * -1.
   */
  math::MatrixXd mass_matrix(
      const math::VectorXd& qpos, std::vector<int>& lambda) const;

  /**
   * @brief Compute the centroidal momentum matrix (CMM) for robot under
   * configuration q.
   *
   * Note that base frame (frame 0) and the CoM frame (frame G) are always set
   * to be parallel.
   *
   * @param[in] qpos A valid configuration (position) vector for robot.
   * @param[out] xpos_com Position of the CoM.
   * @param[out] total_mass Total mass of the robot.
   * @param[out] cmm The matrix that relates joint velocities to the centroidal
   * momentum (6-by-1 vector), i.e. the aggregate momentum of all bodies in the
   * robot projected to the CoM frame.
   */
  void centroidal_momentum_matrix(
      const math::VectorXd& qpos,
      math::Vector3d& xpos_com,
      double& total_mass,
      math::Matrix6X& cmm) const;

  /**
   * @brief Sparsity-preserving Cholesky decomposition for rigid body tree
   * inertia matrix ONLY.
   *
   * The first input argument, @c inertia_matrix , is a valid vNum x vNum
   * inertia matrix of a rigid body tree, where vNum is the number of velocity
   * variables. The second input, @c lambda , is a vNum x 1 vector. lambda(i)
   * corrsponds to the index of the last non-zero of row i below the main
   * diagonal. The output, @c L , is a lower trangular matrix that preserves the
   * sparsity of inertiaMatrix, and L'*L = H.
   */
  static math::MatrixXd special_cholesky(
      const math::MatrixXd& inertia_matrix, const std::vector<int>& lambda);

  // A nonempty string must be provided upon object construction
  std::string m_name;

  // Number of bodies including the base
  int m_num_bodies;

  // Bodies including the base
  std::vector<kdl::RigidBodyPtr> m_bodies;

  // Actuators
  std::vector<kdl::ActuatorPtr> m_actuators;

  // Joint coupling constriants
  std::vector<kdl::CouplingPtr> m_couplings;

  // Joint coupling constriants
  std::vector<kdl::TendonPtr> m_tendons;

  // A map that keeps track of joint position indices in the configuration (q)
  // vector
  std::vector<std::pair<int, int>> m_position_dof_map;

  // A map that keeps track of joint velocity(acceleration) indices in the qdot
  // vector
  std::vector<std::pair<int, int>> m_velocity_dof_map;

  // Number of rigid bodies with non-fixed joint in the robot
  int m_num_non_fixed_bodies;

  // Robot's position variable number
  int m_position_number;

  // Robot's velocity variable number, real DoF
  int m_velocity_number;

  // Robot's actuator (control) number
  int m_actuator_number;

  // Gravitational Acceleration the robot experiences
  math::Vector3d m_gravity;

  friend RigidBody;
  friend Joint;
};

} // namespace crobot::kdl
