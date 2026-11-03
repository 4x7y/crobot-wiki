#include "crobot/common/assert.hpp"
#include "crobot/kdl/site.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
Site::Site(
    const std::string& name,
    Type type,
    ConstRigidBodyPtr body,
    const math::Vector3d& bpos,
    const math::Quaternion& quat,
    const math::Vector3d& size)
  : m_name(name),
    m_type(type),
    m_body(body),
    m_bpos(bpos),
    m_quat(quat),
    m_size(size) {
  CROBOT_ASSERT_ERROR(body != nullptr, "body is null");
}

//==============================================================================
ConstSitePtr Site::MakeShared(
    const std::string& name,
    Type type,
    ConstRigidBodyPtr body,
    const math::Vector3d& bpos,
    const math::Quaternion& quat,
    const math::Vector3d& size) {
  return std::make_shared<Site>(name, type, body, bpos, quat, size);
}

//==============================================================================
const std::string& Site::get_name() const {
  return m_name;
}

//==============================================================================
const math::Vector3d& Site::get_bpos() const {
  return m_bpos;
}

//==============================================================================
const math::Quaternion& Site::get_quat() const {
  return m_quat;
}

//==============================================================================
math::Matrix4d Site::get_tform() const {
  math::Matrix4d tform    = m_quat.as_tform();
  tform.block<3, 1>(0, 3) = m_bpos;
  return tform;
}

//==============================================================================
const math::Vector3d& Site::get_size() const {
  return m_size;
}

//==============================================================================
ConstRigidBodyPtr Site::get_body() const {
  return m_body;
}
