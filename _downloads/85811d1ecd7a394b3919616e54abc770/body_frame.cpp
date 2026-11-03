
#include "crobot/common/assert.hpp"
#include "crobot/kdl/body_contact.hpp"
using namespace crobot;
using namespace kdl;

//==============================================================================
BodyFrame::BodyFrame(ConstRigidBodyPtr body, const math::Matrix4d& tform)
  : m_body(body), m_tform(tform) {
}

//==============================================================================
BodyFrame::BodyFrame(
    ConstRigidBodyPtr body,
    const math::Vector3d& bpos,
    const math::Vector3d& normal)
  : m_body(body), m_tform(math::Matrix4d::Zero()) {
  math::Matrix3d rotm;
  rotm.leftCols(2)  = normal.transpose().fullPivLu().kernel();
  rotm.rightCols(1) = normal;
  if (rotm.determinant() < 0.) {
    rotm.col(1) = -rotm.col(1).eval();
  }
  CROBOT_ASSERT_ERROR(
      math::is_zero(rotm.determinant() - 1.0),
      "The determinant of a rotation matrix must be 1.");

  m_tform.block<3, 3>(0, 0) = rotm;
  m_tform.block<3, 1>(0, 3) = bpos;
}

//==============================================================================
kdl::ConstRigidBodyPtr BodyFrame::get_body() const {
  return m_body;
}

//==============================================================================
const math::Matrix4d& BodyFrame::get_tform() const {
  return m_tform;
}

//==============================================================================
const math::Vector3d BodyFrame::get_bpos() const {
  return m_tform.block<3, 1>(0, 3);
}

//==============================================================================
const math::Matrix3d BodyFrame::get_rotm() const {
  return m_tform.block<3, 3>(0, 0);
}

//==============================================================================
void BodyFrame::set_tform(const math::Matrix4d& tform) {
  m_tform = tform;
}
