#pragma once

#include <mujoco/mjmodel.h>

#include "crobot/kdl/rigid_body.hpp"
#include "crobot/simulation/mj_joint.hpp"
#include "crobot/simulation/mj_site.hpp"

namespace crobot::simulation {

class MJRBTree;
class MJRigidBody;
using MJRigidBodyPtr      = std::shared_ptr<MJRigidBody>;
using ConstMJRigidBodyPtr = std::shared_ptr<const MJRigidBody>;

class MJRigidBody : public kdl::RigidBody {
public:
  static MJRigidBodyPtr MakeShared(
      const mjModel* model, int id, MJJointPtr joint);

  static MJRigidBodyPtr MakeSharedDummy(
      const mjModel* model, const std::string& name, MJJointPtr joint);

  MJRigidBody(const mjModel* model, int id, MJJointPtr joint);

  MJRigidBody(const mjModel* model, const std::string& name, MJJointPtr joint);

  /**
   * @brief Check if the current body is a dummy body.
   *
   * Dummy body is created when converting a Mujoco body with a FREE joint to a
   * combination of three dummy bodies with a PRISMATIC joint each and a regular
   * body with a BALL joint.
   */
  bool is_dummy_body() const;

  /**
   * @brief Get a site with the given name.
   */
  MJSite::ConstPtr get_site(const std::string& name) const;

  /**
   * @brief Add a site to the body.
   */
  void add_site(MJSite::ConstPtr site);

  /**
   * @brief Add a site to the body.
   *
   * @param site_name Name of the site.
   * @param bpos Position of the site in the body frame.
   * @param quat Orientation of the site in the body frame.
   */
  void add_site(
      const std::string& site_name,
      kdl::Site::Type type,
      const math::Vector3d& bpos,
      const math::Quaternion& quat,
      const math::Vector3d& size);

  // Documentation inherited
  using kdl::RigidBody::get_xpos;

  math::Vector3d get_xpos() const;

  math::Vector3d get_xpos(const math::Vector3d& bpos) const;

  // Documentation inherited
  using kdl::RigidBody::get_xquat;

  math::Quaternion get_xquat() const;

  // Documentation inherited
  using kdl::RigidBody::get_xmat;

  math::Matrix3d get_xmat() const;

  math::Matrix3d get_xmat(const math::Matrix3d& rotm) const;

  // Documentation inherited
  using kdl::RigidBody::get_xform;

  math::Matrix4d get_xform() const;

  // Documentation inherited
  using kdl::RigidBody::get_jacb;

  math::Jacobian get_jacb() const;

  math::Jacobian get_jacb_at_xpos(const math::Vector3d& xpos) const;

  math::LinearJacobian get_jacp_at_xpos(const math::Vector3d& xpos) const;

  /**
   * @brief Get the xvel (xvelr, xvelp) of the body
   *
   * @return math::Vector3d
   */
  math::Vector6d get_xvel_mj() const;

private:
  const mjModel* m_model;

  int m_mjid;

  friend class MJRBTree;
};

} // namespace crobot::simulation

