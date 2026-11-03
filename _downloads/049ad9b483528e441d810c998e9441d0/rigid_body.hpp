#pragma once

#include <memory>
#include <string>

#include "crobot/collision/collision_body.hpp"
#include "crobot/kdl/rigid_body_joint.hpp"
#include "crobot/kdl/site.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"

namespace crobot::kdl {

class RigidBodyTree;
class RigidBody;
using RigidBodyPtr          = std::shared_ptr<RigidBody>;
using ConstRigidBodyPtr     = std::shared_ptr<const RigidBody>;
using ConstRigidBodyTreePtr = std::shared_ptr<const RigidBodyTree>;

enum class JacobianType
{
  WITH_RESPECT_TO_QPOS_DERIVATIVE,
  WITH_RESPECT_TO_QVEL,
};

/**
 * @brief Rigid body class.
 *
 * A rigid body is the building block for any tree-structured robot manipulator.
 * Each rigidBody has a rigidBodyJoint object attached to it that defines how
 * the rigid body can move. Rigid bodies are assembled into a tree-structured
 * robot model using rigidBodyTree.
 *
 * Set a joint object to the Joint property before calling addBody to add the
 * rigid body to the robot model. When a rigid body is in a rigid body tree, you
 * cannot directly modify its properties because it corrupts the relationships
 * between bodies. Use replaceJoint to modify the entire tree structure.
 */
class RigidBody : public collision::CollisionBody {
public:
  static RigidBodyPtr MakeShared(const std::string& name);

  static RigidBodyPtr MakeShared(const std::string& name, JointPtr joint);

  /**
   * @brief Construct a new Rigid Body object with FIXED joint.
   *
   * @param name The name of the rigid body.
   */
  explicit RigidBody(const std::string& name);

  /**
   * @brief Construct a new Rigid Body object with a given joint.
   *
   * @param name The name of the rigid body.
   * @param joint The joint object attached to the rigid body.
   */
  RigidBody(const std::string& name, JointPtr joint);

  /**
   * @brief Copy constructor.
   */
  RigidBody(const RigidBody& other);

  /**
   * @brief Destroy the Rigid Body object.
   */
  ~RigidBody() = default;

  /**
   * @brief Get the rigid body's name.
   */
  virtual const std::string& get_name() const;

  /**
   * @brief Get the RigidBodyTree this body associated with. Return nullptr if
   * this is a free body.
   */
  virtual ConstRigidBodyTreePtr get_rbtree() const;

  /**
   * @brief Set the inertia of the rigid body relative to the body frame.
   *
   * @param inertia The 6 independent elements are specified in the order of
   * [Ixx Iyy Izz Iyz Ixz Ixy]. Unit: kilogram-meter-squared (kg*m^2)
   */
  virtual void set_inertia(const math::Vector6d& inertia);

  /**
   * @brief Set the inertia of the rigid body relative to the body frame.
   *
   * @param inertia The inertia matrix. Unit: kilogram-meter-squared (kg*m^2)
   */
  virtual void set_inertia(const math::Matrix3d& inertia);

  /**
   * @brief Set the mass of the rigid body.
   *
   * @param mass The mass value. Unit: kilogram (kg)
   */
  virtual void set_mass(double mass);

  /**
   * @brief Get the mass of the rigid body.
   *
   * @return The mass value. Unit: kilogram (kg)
   */
  virtual double get_mass() const;

  /**
   * @brief Set the center of mass (COM).
   *
   * @param com Center of mass coordinates with respect to the body frame.
   */
  virtual void set_com(const math::Vector3d& com);

  /**
   * @brief Get the center of mass (COM).
   *
   * @return Center of mass coordinates with respect to the body frame.
   */
  virtual math::Vector3d get_com() const;

  /**
   * @brief Get the inertia of the rigid body relative to the body frame.
   *
   * The 6 independent elements of the inertia are specified in the order of
   * [Ixx Iyy Izz Iyz Ixz Ixy]. Unit: kilogram-meter-squared (kg*m^2)
   */
  virtual math::Vector6d get_inertia_vector() const;

  /**
   * @brief Get the inertia matrix of the rigid body relative to the body frame.
   *
   * Unit: kilogram-meter-squared (kg*m^2)
   */
  virtual math::Matrix3d get_inertia_matrix() const;

  /**
   * @brief Get the inertia matrix of the rigid body relative to the
   * center-of-mass (CoM) frame.
   *
   * @param[in] rotm The orientation of the CoM frame in the body frame {B}.
   * Defaults to the identity matrix.
   *
   * Unit: kilogram-meter-squared (kg*m^2)
   */
  virtual math::Matrix3d get_inertia_matrix_at_com(
      const math::Matrix3d& rotm = math::Matrix3d::Identity()) const;

  /**
   * @brief Get the wrench needed to move the rigid body with the given
   * acceleration according to the Newton-Euler equation in the
   * world frame. (inverse dynamics)
   */

  virtual math::Vector6d inverse_dynamics_body_stationary_frame(
      const math::Vector3d& bvelr,
      const math::Vector3d& baccp,
      const math::Vector3d& baccr) const;

  /**
   * @brief Check if this is the base body of the associated RigidBodyTree.
   */
  virtual bool is_base() const;

  /**
   * @brief Get the joint configuration indices in the RigidBodyTree's qpos.
   */
  virtual std::pair<int, int> get_qpos_addr() const;

  /**
   * @brief Get the joint velocity/acceleration indices in the RigidBodyTree's
   * qvel.
   */
  virtual std::pair<int, int> get_qvel_addr() const;

  /**
   * @brief Get the indexes of the qpos variables of the RigidBodyTree that
   * determine the forward kinematics of the body.
   */
  virtual std::vector<int> get_support_qpos_addr() const;

  /**
   * @brief Get the indexes of the qvel variables of the RigidBodyTree that
   * determine the jacobian matrix of the body.
   */
  virtual std::vector<int> get_support_qvel_addr() const;

  /**
   * @brief Get all the bodies that on the path from a given body to the base in
   * the RigidBodyTree. The joint angles of these bodies determine the forward
   * kinematics of the queried body.
   */
  virtual std::vector<ConstRigidBodyPtr> get_support_movable_bodies() const;

  /**
   * @brief Get the world frame position of the body frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   */
  virtual math::Vector3d get_xpos(
      const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the world frame position of a point in the body frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param bpos Position of the point in the body frame.
   */
  virtual math::Vector3d get_xpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Vector3d& bpos) const;

  /**
   * @brief Get the orientation of the body frame in the world frame,
   * represented as a quaternion.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   */
  virtual math::Quaternion get_xquat(
      const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the orientation of the body frame in the world frame,
   * represented as a rotation matrix.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   */
  virtual math::Matrix3d get_xmat(
      const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the orientation of a frame that attached to the body in the
   * world frame, represented as a rotation matrix.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param rotm Orientation of the frame in the body frame, given as a rotation
   * matrix.
   */
  virtual math::Matrix3d get_xmat(
      const std::vector<math::Matrix4d>& tforms,
      const math::Matrix3d& rotm) const;

  /**
   * @brief Get the position and orientation of the body in the world frame,
   * represented as a homogeneous transformation matrix.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   */
  virtual math::Matrix4d get_xform(
      const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the velocity of the body in the world frame, represented as a
   * Twist vector.
   *
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param qvel Joint velocity vector.
   * @return math::Vector6d Twist vector that stacks the angular and linear
   * velocities.
   */
  virtual math::Vector6d get_xvel(
      const math::Jacobian& jacb, const math::VectorXd& qvel) const;

  /**
   * @brief Get the acc of the body in the world frame, represented as a
   * Twist vector.
   *
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param qvel Joint acc vector.
   * @return math::Vector6d Twist vector that stacks the angular and linear
   * acc.
   */
  virtual math::Vector6d get_xacc(
      const math::Jacobian& jacb, const math::VectorXd& qacc) const;

  /**
   * @brief Get the velocity of a body point in the world frame, represented as
   * a Twist vector.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param qvel Joint velocity vector.
   * @param bpos Position of the point in the body frame.
   * @return math::Vector6d Twist vector that stacks the angular and linear
   * velocities.
   */
  virtual math::Vector6d get_xvel_at_bpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::VectorXd& qvel,
      const math::Vector3d& bpos) const;

  /**
   * @brief Get the acc of a body point in the world frame, represented as
   * a Twist vector.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param qvel Joint acc vector.
   * @param bpos Position of the point in the body frame.
   * @return math::Vector6d Twist vector that stacks the angular and linear
   * velocities.
   */
  virtual math::Vector6d get_xacc_at_bpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::VectorXd& qacc,
      const math::Vector3d& bpos) const;

  /**
   * @brief Get the Jacobian of the body frame with respect to the world frame.
   * The output Jacobian is a 6 by ndof matrix, supposed to left mulitple a
   * robot qvel vector.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @return math::Jacobian A 6 by n matrix, where n is the number of degrees of
   * freedom. The first three rows are the angular velocity Jacobian, and the
   * last three rows are the linear velocity Jacobian.
   */
  virtual math::Jacobian get_jacb(
      const std::vector<math::Matrix4d>& tforms) const;

  /**
   * @brief Get the Jacobian of the body frame with respect to the world frame.
   *
   * @param qpos Robot configuration.
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param type The type of Jacobian to compute, either with respect to the
   * qpos derivative or qvel. The two options lead to different Jacobians when
   * the joint type is BALL joint, as qpos is represented by a quaternion while
   * qvel is represented by a angular velocity vector.
   * @return math::Jacobian A 6 by n matrix, where n is the number of degrees of
   * freedom. The first three rows are the angular velocity Jacobian, and the
   * last three rows are the linear velocity Jacobian.
   */
  virtual math::Jacobian get_jacb(
      const math::VectorXd& qpos,
      const std::vector<math::Matrix4d>& tforms,
      JacobianType type = JacobianType::WITH_RESPECT_TO_QVEL) const;

  /**
   * @brief Get the Jacobian of the body fixed point (given body frame
   * coordinates) with respect to the world frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param bpos Position of the point in the body frame.
   * @return math::Jacobian A 6 by n matrix, where n is the number of degrees
   * of freedom. The first three rows are the angular velocity Jacobian, and the
   * last three rows are the linear velocity Jacobian.
   */
  virtual math::Jacobian get_jacb_at_bpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::Vector3d& bpos) const;

  /**
   * @brief Get the Jacobian of the body fixed point (given world frame
   * coordinates) with respect to the world frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param xpos Position of the point in the world frame.
   * @return math::Jacobian A 6 by n matrix, where n is the number of degrees
   * of freedom. The first three rows are the angular velocity Jacobian, and the
   * last three rows are the linear velocity Jacobian.
   */
  virtual math::Jacobian get_jacb_at_xpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::Vector3d& xpos) const;

  /**
   * @brief Get the linear geometric Jacobian of the body frame with respect to
   * the world frame.
   *
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @return math::LinearJacobian A 3 by n matrix, where n is the number of
   * degrees of freedom.
   */
  virtual math::LinearJacobian get_jacp(const math::Jacobian& jacb) const;

  /**
   * @brief Get the linear geometric Jacobian of the body fixed point with
   * respect to the world frame. The body fixed point is specified in the body
   * frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param bpos Position of the point in the body frame.
   * @return math::LinearJacobian A 3 by n matrix, where n is the number of
   * degrees of freedom.
   */
  virtual math::LinearJacobian get_jacp_at_bpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::Vector3d& bpos) const;

  /**
   * @brief Get the linear geometric Jacobian of the body fixed point with
   * respect to the world frame. The body fixed point is specified in the world
   * frame.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param xpos Position of the point in the world frame.
   * @return math::LinearJacobian A 3 by n matrix, where n is the number of
   * degrees of freedom.
   */
  virtual math::LinearJacobian get_jacp_at_xpos(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::Vector3d& xpos) const;

  /**
   * @brief Get the angular geometric Jacobian of the body frame with respect to
   * the world frame.
   *
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @return math::AngularJacobian A 3 by n matrix, where n is the number of
   * degrees of freedom.
   */
  virtual math::AngularJacobian get_jacr(const math::Jacobian& jacb) const;

  /**
   * @brief Get the Jacobian of a body frame orientation vector.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param bvec Orientation vector in the body frame.
   * @return math::LinearJacobian
   */
  virtual math::LinearJacobian get_bvec_jacb(
      const std::vector<math::Matrix4d>& tforms,
      const math::AngularJacobian& jacr,
      const math::Vector3d bvec) const;

  /**
   * @brief Get the joint associated with this body. Return null if no joints
   * have been specified.
   */
  virtual ConstJointPtr get_joint() const;

  /**
   * @brief Get a site with the given name.
   */
  ConstSitePtr get_site(const std::string& name) const;

  /**
   * @brief Add a site to the body.
   */
  void add_site(ConstSitePtr site);

  /**
   * @brief Add a site to the body.
   *
   * @param site_name Name of the site.
   * @param bpos Position of the site in the body frame.
   * @param quat Orientation of the site in the body frame.
   */
  void add_site(
      const std::string& site_name,
      Site::Type type,
      const math::Vector3d& bpos,
      const math::Quaternion& quat,
      const math::Vector3d& size);

  /**
   * @brief Get the <bfrc, btorq> in order to achieve the given acceleration
   * through inverse dynamics.
   *
   * @param tforms Transformation matrices of all the body frames that are
   * obtained from the RigidBodyTree's forward kinematics evaluated at a certain
   * joint configuration.
   * @param jacb Jacobian of the body frame with respect to the world frame.
   * @param qvel Joint velocities.
   * @param xaccp body frame's translational acceleration in the world
   * frame
   * @param xaccr body frame's rotational acceleration in the world frame
   * @return math::Vector6d wrench in the world frame
   */
  math::Vector6d get_ext_bfrc_btorq_with_bacc(
      const std::vector<math::Matrix4d>& tforms,
      const math::Jacobian& jacb,
      const math::VectorXd& qvel,
      const math::Vector3d baccp,
      const math::Vector3d baccr) const;

  math::Matrix6d get_spatial_inertia_body_stationary_frame() const;

  math::Vector6d get_fictitious_bf_btau(const math::Vector3d& bvelr) const;

protected:
  /**
   * @brief Form the 6x6 spatial inertia matrix.
   *
   * @param mass Mass of the rigid body
   * @param com Center of mass of the rigid body relative to body frame
   * @param inertia 3x3 inertia tensor of the rigid body relative to body frame
   */
  static math::Matrix6d spatial_inertia(
      double mass, const math::Vector3d& com, const math::Matrix3d& inertia);

  /**
   * @brief Transforms the 3x3 inertia matrix @p inertia_com from the
   * center-of-mass (CoM) frame {I} to the target frame {O}. The current CoM
   * frame {I} is related to the target frame {O} through ( @p rotm, @p p )
   * pair.
   *
   * @param[in] inertia_com The inertia matrix in the CoM frame {I}.
   * @param[in] mass The mass of the body.
   * @param[in] rotm The orientation of frame {I} in frame {O}.
   * @param[in] p The position of the origin of frame {I}, relative to frame
   * {O}. This is equivalent to the center of mass (CoM) vector.
   * @return The inertia matrix in the target frame {O}.
   */
  static math::Matrix3d transform_inertia_com2target(
      const math::Matrix3d& inertia_com,
      double mass,
      const math::Matrix3d& rotm,
      const math::Vector3d& p);

  /**
   * @brief Transforms the 3x3 inertia matrix @p inertia from a body frame {B}
   * to the center-of-mass (CoM) frame {I} that locates at the center of mass of
   * the body. The CoM frame {I} is related to the body frame {B} through
   * ( @p rotm, @p p ) pair represented in the body frame.
   *
   * @param[in] inertia The inertia matrix in the body frame {B}.
   * @param[in] mass The mass of the body.
   * @param[in] rotm The orientation of frame {I} in frame {B}.
   * @param[in] p The position of the origin of frame {I}, relative to frame
   * {B}. This is equivalent to the center of mass (CoM) vector.
   * @return The inertia matrix in the CoM frame {I}.
   */
  static math::Matrix3d transform_inertia_body2com(
      const math::Matrix3d& inertia,
      double mass,
      const math::Matrix3d& rotm,
      const math::Vector3d& p);

  /// Name of the body.
  ///
  /// A nonempty string must be provided upon object construction.
  std::string m_name;

  /// The joint associated with the rigid body
  ///
  /// Default: A RigidBodyJoint object with 'fixed' joint type and
  /// '<bodyname>_jnt' joint name.
  kdl::JointPtr m_joint;

  /// Mass of the rigid body.
  ///
  /// Body mass cannot be negative. Unit: kilogram (kg).
  ///
  /// Default: 1
  double m_mass;

  /// Center of mass vector of the rigid body, relative to the body frame. Unit:
  /// meter (m).
  math::Vector3d m_com;

  /// Inertia of the rigid body relative to the body frame.
  ///
  /// Unit: kilogram-meter-squared (kg*m^2)
  ///
  /// Default: Matrix3d::Identity()
  math::Matrix3d m_inertia;

  /// 6x6 spatial inertia matrix.
  math::Matrix6d m_spatial_inertia;

  /// RigidBodyTree that the body associated with.
  kdl::ConstRigidBodyTreePtr m_rbtree;

  /// Parent body of the body.
  ///
  /// This property will be updated automatically once the body is  added to a
  /// robot. By default it's a nullptr.
  kdl::ConstRigidBodyPtr m_parent;

  /// Children of the body.
  ///
  /// This property will be updated automatically once the body is added to a
  /// robot. By default it's a empty vector.
  std::vector<kdl::RigidBodyPtr> m_children;

  /// Index of this body in an associated RigidBodyTree.
  ///
  /// Default: -1
  int m_index;

  /// Sites
  std::vector<ConstSitePtr> m_sites;

  friend class RigidBodyTree;
};

} // namespace crobot::kdl
