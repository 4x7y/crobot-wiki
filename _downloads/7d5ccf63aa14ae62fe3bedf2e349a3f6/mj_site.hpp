#pragma once

#include "crobot/kdl/site.hpp"

namespace crobot::simulation {

class MJRigidBody;
using MJRigidBodyPtr      = std::shared_ptr<MJRigidBody>;
using ConstMJRigidBodyPtr = std::shared_ptr<const MJRigidBody>;

class MJSite : public kdl::Site {
public:
  using Ptr      = std::shared_ptr<MJSite>;
  using ConstPtr = std::shared_ptr<const MJSite>;

  /**
   * @brief Construct a new Site object
   *
   * @param name The name of the site
   * @param type Type of the site
   * @param body The body to which this site is attached
   * @param bpos The position of the site in the body frame
   * @param quat The orientation of the site in the body frame
   * @param size Size of the site
   */
  MJSite(
      const std::string& name,
      kdl::Site::Type type,
      ConstMJRigidBodyPtr body,
      const math::Vector3d& bpos,
      const math::Quaternion& quat,
      const math::Vector3d& size);

  /**
   * @brief Get the body to which this site is attached.
   *
   * @return ConstMJRigidBodyPtr
   */
  ConstMJRigidBodyPtr get_body() const;

  /**
   * @brief Get the position of the site in the world frame.
   *
   * @return math::Vector3d Position of the site
   */
  math::Vector3d get_xpos() const;

  /**
   * @brief Get the orientation of the site in the world frame, expressed as a
   * quaternion.
   *
   * @return math::Quaternion Quaternion representing the orientation of the
   * site
   */
  math::Quaternion get_xquat() const;

  /**
   * @brief Get the orientation of the site in the world frame, expressed as a
   * 3x3 rotation matrix.
   *
   * @return math::Matrix3d Rotation matrix representing the orientation of the
   * site.
   */
  math::Matrix3d get_xmat() const;

  /**
   * @brief Get the pose of the site in the world frame, expressed as a 4x4
   * transformation matrix.
   *
   * @return math::Matrix4d Transformation matrix representing the pose of the
   * site.
   */
  math::Matrix4d get_xform() const;
};

} // namespace crobot::simulation

