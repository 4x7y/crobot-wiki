#include "crobot/kdl/frame.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
//                                  Frame                                     //
//==============================================================================
math::Vector3d Frame::get_xpos() const {
  return get_xform().tform2trvec();
}

//==============================================================================
math::Matrix3d Frame::get_xmat() const {
  return get_xform().tform2rotm();
}

//==============================================================================
//                                WorldFrame                                  //
//==============================================================================
WorldFramePtr WorldFrame::MakeShared(const math::Matrix4d& xform) {
  return std::make_shared<WorldFrame>(xform);
}

//==============================================================================
WorldFrame::WorldFrame(const math::Matrix4d& xform)
  : m_xform(xform),
    m_xvel(math::Vector6d::Zero()),
    m_xacc(math::Vector6d::Zero()) {
}

//==============================================================================
math::Matrix4d WorldFrame::get_xform() const {
  return m_xform;
}

//==============================================================================
math::Vector3d WorldFrame::get_xvelp() const {
  return m_xvel.segment(3, 3);
}

//==============================================================================
math::Vector3d WorldFrame::get_xvelr() const {
  return m_xvel.segment(0, 3);
}

//==============================================================================
math::Vector6d WorldFrame::get_xvel() const {
  return m_xvel;
}

//==============================================================================
math::Vector3d WorldFrame::get_xaccp() const {
  return m_xacc.segment(3, 3);
}

//==============================================================================
math::Vector3d WorldFrame::get_xaccr() const {
  return m_xacc.segment(0, 3);
}

//==============================================================================
math::Vector6d WorldFrame::get_xacc() const {
  return m_xacc;
}

//==============================================================================
void WorldFrame::set_xform(const math::Matrix4d& xform) {
  m_xform = xform;
}

//==============================================================================
void WorldFrame::translate(const math::Vector3d& tvec) {
  m_xform.block<3, 1>(0, 3) += tvec;
}

//==============================================================================
void WorldFrame::set_tvec(const math::Vector3d& tvec) {
  m_xform.block<3, 1>(0, 3) = tvec;
}

//==============================================================================
void WorldFrame::set_xvel(const math::VectorXd& xvel) {
  CROBOT_ASSERT_ERROR(xvel.size() == m_xvel.size());
  m_xvel = xvel;
}

//==============================================================================
void WorldFrame::set_xacc(const math::VectorXd& xacc) {
  CROBOT_ASSERT_ERROR(xacc.size() == m_xacc.size());
  m_xacc = xacc;
}

//==============================================================================
void WorldFrame::set_xwrench_ext(const math::Vector6d& xwrench_ext) {
  m_xwrench_ext = xwrench_ext;
}

//==============================================================================
math::Vector6d WorldFrame::get_xwrench_ext() const {
  return m_xwrench_ext;
}
