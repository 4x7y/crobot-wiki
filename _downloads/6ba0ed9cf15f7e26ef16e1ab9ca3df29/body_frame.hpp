#pragma once

#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"

namespace crobot::kdl {

class BodyFrame {
public:
  using Ptr      = std::shared_ptr<BodyFrame>;
  using ConstPtr = std::shared_ptr<const BodyFrame>;

  /**
   * @brief Construct a BodyFrame object for the given body.
   *
   * @param body The body on which the contacts are located.
   * @param tform The pose of the frame in the body's frame. The
   * Z-axis (the third column of rotm) points to the interior of the body
   * object.
   */
  BodyFrame(kdl::ConstRigidBodyPtr body, const math::Matrix4d& tform);

  /**
   * @brief Construct a BodyFrame object for the given body.
   *
   * @param body The body on which the contacts are located.
   * @param bpos The position of the frame in the body's frame.
   * @param rotm The orientation of the frame in the body's frame. The
   * Z-axis (the third column of rotm) points to the interior of the body
   * object.
   */
  BodyFrame(
      kdl::ConstRigidBodyPtr body,
      const math::Vector3d& bpos,
      const math::Vector3d& normal);

  /**
   * @brief Get the body this frame is located on.
   *
   * @return kdl::ConstRigidBodyPtr The body this frame is located on.
   */
  kdl::ConstRigidBodyPtr get_body() const;

  /**
   * @brief Get the pose of the frame in the body frame, represented as
   * a transformation matrix.
   *
   * @return math::Matrix4d Transformation matrix
   */
  const math::Matrix4d& get_tform() const;

  const math::Vector3d get_bpos() const;

  const math::Matrix3d get_rotm() const;

  void set_tform(const math::Matrix4d& tform);

private:
  ConstRigidBodyPtr m_body;
  math::Matrix4d m_tform;
};

} // namespace crobot::kdl

