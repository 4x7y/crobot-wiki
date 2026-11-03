#pragma once

#include <memory>

#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"

namespace crobot::kdl {

class RigidBody;
using ConstRigidBodyPtr = std::shared_ptr<const RigidBody>;
class Site;
using ConstSitePtr = std::shared_ptr<const Site>;

class Site {
  friend RigidBody;

public:
  enum Type
  {
    PLANE = 0, // plane
    HFIELD,    // height field
    SPHERE,    // sphere
    CAPSULE,   // capsule
    ELLIPSOID, // ellipsoid
    CYLINDER,  // cylinder
    BOX,       // box
    MESH,      // mesh
  };

  /**
   * @brief Construct a new Site object
   *
   * @param name The name of the site
   * @param type Type of the site
   * @param bpos The position of the site in the body frame
   * @param quat The orientation of the site in the body frame
   * @param size Size of the site
   */
  Site(
      const std::string& name,
      Type type,
      ConstRigidBodyPtr body,
      const math::Vector3d& bpos,
      const math::Quaternion& quat,
      const math::Vector3d& size);

  /**
   * @brief Make a shared pointer of Site.
   *
   * @param name Name of the site.
   * @param type Type of the site.
   * @param bpos Position of the site in the body frame.
   * @param quat Orientation of the site in the body frame.
   * @param size Size of the site.
   * @return ConstSitePtr Shared pointer of Site.
   */
  static ConstSitePtr MakeShared(
      const std::string& name,
      Type type,
      ConstRigidBodyPtr body,
      const math::Vector3d& bpos,
      const math::Quaternion& quat,
      const math::Vector3d& size);

  /**
   * @brief Get the name of this site.
   *
   * @return const std::string& The name of this site.
   */
  virtual const std::string& get_name() const;

  /**
   * @brief Get the location of this site in the body frame.
   *
   * @return const math::Vector3d& The location of this site in the body frame.
   */
  virtual const math::Vector3d& get_bpos() const;

  /**
   * @brief Get the orientation of this site in the body frame as a quaternion.
   *
   * @return const math::Quaternion& The orientation of this site in the body
   * frame.
   */
  virtual const math::Quaternion& get_quat() const;

  /**
   * @brief Get the pose of the site in the body frame, expressed as a 4x4
   * transformation matrix.
   *
   * @return const math::Matrix4d The pose of the site in the body frame.
   */
  virtual math::Matrix4d get_tform() const;

  /**
   * @brief Get the size of this site.
   *
   * @return const math::Vector3d& The size of this site.
   */
  virtual const math::Vector3d& get_size() const;

  /**
   * @brief Get the body to which this site is attached.
   *
   * @return ConstRigidBodyPtr Shared pointer of the body.
   */
  ConstRigidBodyPtr get_body() const;

private:
  /// name of the site
  std::string m_name;

  /// type of the site
  Type m_type;

  /// body to which this site is attached
  ConstRigidBodyPtr m_body;

  /// local position offset rel. to body
  math::Vector3d m_bpos;

  /// local orientation offset rel. to body
  math::Quaternion m_quat;

  /// size of the site
  math::Vector3d m_size;
};

} // namespace crobot::kdl

