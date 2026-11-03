#include "crobot/kdl/body_contact.hpp"

#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace kdl;
using namespace optimization;

//==============================================================================
BodyContact::BodyContact(
    ContactModel contact_model,
    ConstRigidBodyPtr body1,
    BodyFrame::Ptr frame1,
    ConstRigidBodyPtr body2,
    BodyFrame::Ptr frame2,
    double mu)
  : CollisionContact(contact_model, mu, false),
    m_body1(body1),
    m_body2(body2),
    m_frame1(frame1),
    m_frame2(frame2) {
}

//==============================================================================
math::Matrix6X BodyContact::get_contact_map() const {
  // orientation of the contact frame in the object frame
  math::Matrix3d rotm = m_frame1->get_tform().tform2rotm();
  math::Vector3d bpos = m_frame1->get_tform().tform2trvec();

  math::Matrix6d G = math::Matrix6d::Zero();
  // transform into the body frame
  G.block<3, 3>(0, 0) = rotm;
  G.block<3, 3>(0, 3) = math::Matrix3d::Zero();
  G.block<3, 3>(3, 0) = bpos.skew() * rotm;
  G.block<3, 3>(3, 3) = rotm;

  math::Matrix6X ctc_map = G * get_selection_matrix().transpose();
  return ctc_map;
}

//==============================================================================
ConstRigidBodyPtr BodyContact::get_body1() const {
  return m_body1;
}

//==============================================================================
ConstRigidBodyPtr BodyContact::get_body2() const {
  return m_body2;
}

//==============================================================================
BodyFrame::Ptr BodyContact::get_frame1() const {
  return m_frame1;
}

//==============================================================================
BodyFrame::Ptr BodyContact::get_frame2() const {
  return m_frame2;
}

//==============================================================================
math::Vector3d BodyContact::get_xpos(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d tform = m_frame1->get_tform();
  math::Vector3d bpos  = tform.tform2trvec();
  math::Vector3d xpos  = m_body1->get_xpos(tforms, bpos);
  return xpos;
}

//==============================================================================
math::Vector3d BodyContact::get_xpos1(
    const std::vector<math::Matrix4d>& tforms) const {
  return get_xpos(tforms);
}

//==============================================================================
math::Vector3d BodyContact::get_xpos2(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d tform = m_frame2->get_tform();
  math::Vector3d bpos  = tform.tform2trvec();
  math::Vector3d xpos  = m_body2->get_xpos(tforms, bpos);
  return xpos;
}

//==============================================================================
math::Matrix3d BodyContact::get_xmat(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d tform = m_frame1->get_tform();
  math::Matrix3d bmat  = tform.tform2rotm();
  math::Matrix3d xmat  = m_body1->get_xmat(tforms, bmat);
  return xmat;
}

//==============================================================================
math::Matrix3d BodyContact::get_xmat1(
    const std::vector<math::Matrix4d>& tforms) const {
  return get_xmat(tforms);
}

//==============================================================================
math::Matrix3d BodyContact::get_xmat2(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d tform = m_frame2->get_tform();
  math::Matrix3d bmat  = tform.tform2rotm();
  math::Matrix3d xmat  = m_body2->get_xmat(tforms, bmat);
  return xmat;
}

//==============================================================================
math::Vector6d BodyContact::map_cwrench_to_bwrench(
    const math::Vector6d& cwrench) const {

  math::Vector6d bwrench = get_contact_map() * get_selection_matrix() * cwrench;
  return bwrench;
}

//==============================================================================
math::MatrixX6 BodyContact::get_contact_map_inverse() const {
  // // orientation of the contact frame in the object frame
  // math::Matrix3d rotm = m_frame1->get_tform().tform2rotm();
  // math::Vector3d bpos = m_frame1->get_tform().tform2trvec();

  // math::Matrix6d Ginv;
  // // transform into the contact frame
  // Ginv.block<3, 3>(0, 0) = rotm.transpose();
  // Ginv.block<3, 3>(0, 3) = -bpos.skew() * rotm.transpose();
  // Ginv.block<3, 3>(3, 0) = math::Matrix3d::Zero();
  // Ginv.block<3, 3>(3, 3) = rotm.transpose();

  // return get_selection_matrix() * Ginv;

  return math::calculate_jacb_inv(get_contact_map());
}
