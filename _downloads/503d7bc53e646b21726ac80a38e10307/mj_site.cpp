#include "crobot/simulation/mj_rigid_body.hpp"
#include "crobot/simulation/mj_site.hpp"

using namespace crobot;
using namespace simulation;

//==============================================================================
MJSite::MJSite(
    const std::string& name,
    kdl::Site::Type type,
    ConstMJRigidBodyPtr body,
    const math::Vector3d& bpos,
    const math::Quaternion& quat,
    const math::Vector3d& size)
  : kdl::Site(name, type, body, bpos, quat, size) {
}

//==============================================================================
ConstMJRigidBodyPtr MJSite::get_body() const {
  return std::dynamic_pointer_cast<const MJRigidBody>(kdl::Site::get_body());
}

//==============================================================================
math::Vector3d MJSite::get_xpos() const {
  return get_body()->get_xpos(get_bpos());
}

//==============================================================================
math::Quaternion MJSite::get_xquat() const {
  return get_body()->get_xquat() * get_quat();
}

//==============================================================================
math::Matrix3d MJSite::get_xmat() const {
  return get_body()->get_xmat(get_quat().as_rotm());
}

//==============================================================================
math::Matrix4d MJSite::get_xform() const {
  return get_body()->get_xform() * get_tform();
}
