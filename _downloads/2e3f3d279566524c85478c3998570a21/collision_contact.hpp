#pragma once

#include "crobot/kdl/contact.hpp"

namespace crobot::kdl {

class CollisionContact : public Contact {
public:
  using Ptr      = std::shared_ptr<CollisionContact>;
  using ConstPtr = std::shared_ptr<const CollisionContact>;

  CollisionContact(
      ContactModel contact_model = ContactModel::POINT_CONTACT_WITH_FRICTION,
      double mu                  = 1.0,
      bool is_active             = false);

  virtual ~CollisionContact() = default;

  /**
   * @brief Get the contact location in the world frame.
   */
  virtual math::Vector3d get_xpos() const;

  /**
   * @brief Get the contact frame in the world frame.
   *
   * @return The contact frame orientation. The Z-axis (the third column of
   * rotm) points to the interior of the body object.
   */
  virtual math::Matrix3d get_xmat() const;

  /**
   * @brief Check if the contact is established or not.
   */
  virtual bool is_active() const;

  /**
   * @brief Get the contact force in the world frame.
   */
  virtual math::Vector3d get_xfrc() const;

  void update_status(bool is_active);

protected:
  /**
   * @brief Update contact location and force. Contact is updated to active
   * state automatically.
   *
   * @param xpos Position of the cotnact frame in the world frame
   * @param xmat Orientation of the cotnact frame in the world frame
   * @param xfrc Contact force in the world frame
   */
  void update(
      const math::Vector3d& xpos,
      const math::Matrix3d& xmat,
      const math::Vector3d& xfrc);

  /**
   * @brief Update contact location and force. Contact is updated to active
   * state automatically.
   *
   * @param xform Pose of the cotnact frame in the world frame
   * @param xfrc Contact force in the world frame
   */
  void update(const math::Matrix4d& xform, const math::Vector3d& xfrc);

private:
  math::Vector3d m_xfrc;
  math::Matrix4d m_xform;
  bool m_is_active;
};

} // namespace crobot::kdl

