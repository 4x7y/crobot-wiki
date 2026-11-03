#pragma once

#include <string>

#include "crobot/kdl/transmission.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::kdl {

enum class JointType
{
  /// Fixed to the parent body
  FIXED,
  /// Rotation angle (rad) around body-fixed axis
  REVOLUTE,
  /// Sliding distance along body-fixed axis
  PRISMATIC,
  /// Orientation (quat) relative to parent
  BALL,
};

class RigidBody;
class RigidBodyTree;
class Tendon;
class Joint;
using JointPtr      = std::shared_ptr<Joint>;
using ConstJointPtr = std::shared_ptr<const Joint>;

class Joint : public Transmission {
public:
  static JointPtr MakeShared(
      const std::string& name, JointType type = JointType::FIXED);

  Joint(const std::string& name, JointType type);
  virtual ~Joint()    = default;
  Joint(const Joint&) = default;
  Joint& operator=(const Joint&) = default;

  /**
   * @brief Set the name of the joint.
   */
  virtual void set_name(const std::string& name);

  /**
   * @brief Get the name of the joint.
   */
  virtual const std::string& get_name() const;

  /**
   * @brief Get the type of the joint.
   */
  virtual JointType get_type() const;

  /**
   * @brief Get the number of variables used to represent the joint
   * configuration.
   */
  virtual int get_qpos_dim() const;

  /**
   * @brief Get the number of variables used to represent the joint
   * velocity or acceleration.
   */
  virtual int get_qvel_dim() const;

  /**
   * @brief Get the joint configuration indices in the RigidBodyTree
   * configuration vector @c qpos .
   */
  std::vector<std::pair<int, int>> get_qpos_addr() const override;

  /**
   * @brief Get the joint velocity (acceleration) indices in the RigidBodyTree
   * velocity (acceleration) vector @c qvel ( @c qacc ).
   */
  std::vector<std::pair<int, int>> get_qvel_addr() const override;

  /**
   * @brief Get the transmission coefficients. Returns [1].
   */
  std::vector<double> get_transmission_coeffs() const override;

  /**
   * @brief Get the qpos limits of this joint.
   *
   * @return std::pair<double, double> joint limits
   */
  virtual std::pair<math::VectorXd, math::VectorXd> get_qpos_limits() const;

  /**
   * @brief Set the qpos limits of REVOLUTE and PRISMATIC joint.
   *
   * @param lb lower bound of the joint range
   * @param ub upper bound of the joint range
   */
  virtual void set_qpos_limits(double lb, double ub);

  /**
   * @brief Set the qpos limits of this joint.
   *
   * @param lb lower bound of the joint range
   * @param ub upper bound of the joint range
   */
  virtual void set_qpos_limits(
      const math::VectorXd& lb, const math::VectorXd& ub);

  /**
   * @brief Set home position for prismatic/revolute joints.
   */
  virtual void set_home_position(double qpos);

  /**
   * @brief Set home position.
   */
  virtual void set_home_position(const math::VectorXd& qpos);

  /**
   * @brief Generate a random configuration within the limits.
   */
  virtual math::VectorXd random_qpos() const;

  virtual math::VectorXd random_qpos_offset(
      double ref, double offset_limit) const;

  /**
   * @brief Set the joint axis for prismatic/revolute joints.
   */
  virtual void set_axis(const math::Vector3d& axis);

  /**
   * @brief Set fixed transform properties of joint.
   *
   * Sets m_tform_joint2parent to the user-supplied 4x4 homogeneous transform
   * matrix T and m_tform_child2joint to an identity matrix.
   */
  virtual void set_fixed_transform(const math::Matrix4d& tform);

  /**
   * @brief Get the transformaion matrix of the body frame in its parent frame
   * for fixed joints.
   */
  virtual math::Matrix4d tform_body2parent() const;

  /**
   * @brief Get the transformaion matrix of the body frame in its parent frame
   * for prismatic/revolute joints.
   */
  virtual math::Matrix4d tform_body2parent(double qpos) const;

  /**
   * @brief Get the transformaion matrix of the body frame in its parent frame.
   */
  virtual math::Matrix4d tform_body2parent(const math::VectorXd& qpos) const;

protected:
  /**
   * @brief Compute the variable joint transform that is ependent on joint
   * position. @c qpos is the joint position vector.
   */
  math::Matrix4d tform_joint(const math::VectorXd& qpos) const;

  // Currently, supported joint types are: fixed, revolute, prismatic and free.
  JointType m_type;

  // Name of the joint
  std::string m_name;

  // Is the joint associated with a rigid body tree.
  bool m_in_tree;

  // Joint position indices in the configuration (q) vector
  std::pair<int, int> m_qpos_addr;

  // Joint velocity(acceleration) indices in the qdot vector
  std::pair<int, int> m_qvel_addr;

  // Axis of motion for revolute/prismatic joint
  //
  // JointAxis represents the rotation axis of a revolute joint, or the sliding
  // direction of a prismatic joint. This property is only meaningful for
  // revolute or prismatic joints.
  //
  // Default: [0, 0, 0] for fixed joint
  // Default: [0, 0, 1] for revolute/prismatic joint
  math::Vector3d m_axis;

  // Position limits of the revolute/prismatic joint
  //
  // Default: [ -PI,  PI] for revolute joints
  // Default: [-0.5, 0.5] for prismatic joints
  // Default: [-0.5, 0.5; -0.5, 0.5; -0.5; 0.5] for free joints' traslation
  std::pair<math::VectorXd, math::VectorXd> m_limits;

  // Home position of the joint
  //
  // Home position must fall in the range set by PositionLimits
  //
  // Default: [0] for revolute/prismatic joints
  // Default: [1, 0, 0, 0] for ball joints
  // Default: [0, 0, 0, 1, 0, 0, 0] for free joints
  math::VectorXd m_home_position;

  // The first fixed transform property of a joint JointToParentTransform
  // represents the homogeneous transform that converts points originally
  // expressed in the joint predecessor frame to the parent body frame. This
  // property is read-only.
  //
  // Note joint predecessor frame is fixed on the parent body.
  //
  // Default: Matrix4d::Identity()
  math::Matrix4d m_tform_joint2parent;

  // The second fixed transform property of a joint ChildToJointTransform
  // represents the homogeneous transform that converts points originally
  // expressed in the child body frame to the joint successor frame. This
  // property is read-only.
  //
  // Note the rigid body who owns the joint is the child body in this context.
  // And the joint successor frame is fixed on the child body.
  //
  // Default: Matrix4d::Identity()
  math::Matrix4d m_tform_child2joint;

  // Motion subspace
  math::Matrix6X m_motion_subspace;

  friend class Tendon;
  friend class RigidBody;
  friend class RigidBodyTree;
};

} // namespace crobot::kdl
