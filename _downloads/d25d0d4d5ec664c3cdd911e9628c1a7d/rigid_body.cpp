#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rigid_body_joint.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
RigidBodyPtr RigidBody::MakeShared(const std::string& name) {
  return std::make_shared<RigidBody>(name);
}

//==============================================================================
RigidBodyPtr RigidBody::MakeShared(const std::string& name, JointPtr joint) {
  return std::make_shared<RigidBody>(name, std::move(joint));
}

//==============================================================================
RigidBody::RigidBody(const std::string& name)
  : RigidBody(name, Joint::MakeShared(name + "_jnt", JointType::FIXED)) {
}

//==============================================================================
RigidBody::RigidBody(const std::string& name, JointPtr joint)
  : CollisionBody(),
    m_name(name),
    m_joint(joint),
    m_mass(1.),
    m_com(math::Vector3d::Zero()),
    m_index(-1) {
  m_children.reserve(100);
  m_inertia         = math::Matrix3d::Identity();
  m_spatial_inertia = spatial_inertia(m_mass, m_com, m_inertia);
}

//==============================================================================
RigidBody::RigidBody(const RigidBody& other) : collision::CollisionBody(other) {
  m_name     = other.m_name;
  m_joint    = std::make_shared<Joint>(*other.m_joint);
  m_mass     = other.m_mass;
  m_com      = other.m_com;
  m_index    = other.m_index;
  m_children = other.m_children;
  m_inertia  = other.m_inertia;
  m_sites    = std::vector<ConstSitePtr>();
  m_sites.reserve(other.m_sites.size());
  for (std::size_t i = 0; i < m_sites.size(); i++) {
    ConstSitePtr site = std::make_shared<Site>(*other.m_sites[i]);
    m_sites.push_back(site);
  }
}

//==============================================================================
const std::string& RigidBody::get_name() const {
  return m_name;
}

//==============================================================================
ConstRigidBodyTreePtr RigidBody::get_rbtree() const {
  return m_rbtree;
}

//==============================================================================
void RigidBody::set_inertia(const math::Vector6d& I) {
  // I = [Ixx Iyy Izz Iyz Ixz Ixy]
  // clang-format off
  m_inertia = (math::Matrix3d() << 
      I(0), I(5), I(4),
      I(5), I(1), I(3),
      I(4), I(3), I(2)).finished();
  // clang-format on
  m_spatial_inertia = spatial_inertia(m_mass, m_com, m_inertia);
}

//==============================================================================
void RigidBody::set_inertia(const math::Matrix3d& I) {
  m_inertia         = I;
  m_spatial_inertia = spatial_inertia(m_mass, m_com, m_inertia);
}

//==============================================================================
void RigidBody::set_mass(double mass) {
  m_mass            = mass;
  m_spatial_inertia = spatial_inertia(m_mass, m_com, m_inertia);
}

//==============================================================================
double RigidBody::get_mass() const {
  return m_mass;
}

//==============================================================================
void RigidBody::set_com(const math::Vector3d& com) {
  m_com             = com;
  m_spatial_inertia = spatial_inertia(m_mass, m_com, m_inertia);
}

//==============================================================================
math::Vector3d RigidBody::get_com() const {
  return m_com;
}

//==============================================================================
math::Vector6d RigidBody::get_inertia_vector() const {
  CROBOT_ASSERT_ERROR(m_index != 0, "No such property for base.");
  return (math::Vector6d() << m_inertia(0, 0),
          m_inertia(1, 1),
          m_inertia(2, 2),
          m_inertia(1, 2),
          m_inertia(0, 2),
          m_inertia(0, 1))
      .finished();
}

//==============================================================================
math::Matrix3d RigidBody::get_inertia_matrix() const {
  CROBOT_ASSERT_ERROR(m_index != 0, "No such property for base.");
  return m_inertia;
}

//==============================================================================
math::Matrix3d RigidBody::get_inertia_matrix_at_com(
    const math::Matrix3d& rotm) const {
  return transform_inertia_body2com(m_inertia, m_mass, rotm, get_com());
}

//==============================================================================
math::Vector6d RigidBody::inverse_dynamics_body_stationary_frame(
    const math::Vector3d& bvelr,
    const math::Vector3d& baccp,
    const math::Vector3d& baccr) const {
  CROBOT_ASSERT_ERROR(
      !is_base(),
      "The base of a RigidBodyTree does not have inverse dynamics.");

  math::Vector6d bfrc_btorq = math::Vector6d::Zero();

  math::Vector6d bacc;
  bacc << baccp, baccr;

  // Center of mass in the body stationary frame (instantaneously located at the
  // body frame and oriented the same way).
  math::Matrix3d s_bcom  = m_com.skew();
  math::Matrix3d s_bcom2 = s_bcom * s_bcom;

  // Spatial inertia matrix in the body stationary frame.
  math::Matrix6d spatialI = math::Matrix6d::Zero();

  // Inertia matrix represented in the inertial frame (located at the center of
  // mass and oriented the same as the body frame).
  math::Matrix3d I_cm = get_inertia_matrix_at_com();

  spatialI.block<3, 3>(0, 0) = m_mass * math::Matrix3d::Identity();
  spatialI.block<3, 3>(0, 3) = -m_mass * s_bcom;
  spatialI.block<3, 3>(3, 0) = m_mass * s_bcom;
  spatialI.block<3, 3>(3, 3) = I_cm - m_mass * s_bcom2;

  // Fictitious force
  math::Matrix3d s_bvelr = bvelr.skew(); // [omega_dot]x
  math::Vector6d fictF   = math::Vector6d::Zero();
  fictF << m_mass * s_bvelr * s_bvelr * m_com,
      s_bvelr * (I_cm - m_mass * s_bcom2) * bvelr; //

  bfrc_btorq = spatialI * bacc + fictF;

  return bfrc_btorq;
}

//==============================================================================
bool RigidBody::is_base() const {
  return m_index == 0;
}

//==============================================================================
std::pair<int, int> RigidBody::get_qpos_addr() const {
  CROBOT_ASSERT_ERROR(
      m_index != 0, "The base of a RigidBodyTree does not have qpos addrs.");
  CROBOT_ASSERT_ERROR(
      m_index > 0,
      "This body (%s) does not belong to a RigidBodyTree.",
      m_name.c_str());
  return m_rbtree->m_position_dof_map[m_index];
}

//==============================================================================
std::pair<int, int> RigidBody::get_qvel_addr() const {
  CROBOT_ASSERT_ERROR(
      m_index != 0, "The base of a RigidBodyTree does not have qvel addrs.");
  CROBOT_ASSERT_ERROR(
      m_index > 0,
      "This body \"%s\" has not been added to a RigidBodyTree.",
      m_name.c_str());
  CROBOT_ASSERT_ERROR(m_joint != nullptr, "The body does not have a joint.");
  return m_rbtree->m_velocity_dof_map[m_index];
}

//==============================================================================
std::vector<int> RigidBody::get_support_qpos_addr() const {
  return m_rbtree->get_support_qpos_addr(m_index);
}

//==============================================================================
std::vector<int> RigidBody::get_support_qvel_addr() const {
  return m_rbtree->get_support_qvel_addr(m_index);
}

//==============================================================================
std::vector<ConstRigidBodyPtr> RigidBody::get_support_movable_bodies() const {
  return m_rbtree->get_support_movable_bodies(m_index);
}

//==============================================================================
math::Vector3d RigidBody::get_xpos(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d xform = m_rbtree->get_transform(tforms, m_index);
  return xform.tform2trvec();
}

//==============================================================================
math::Vector3d RigidBody::get_xpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Vector3d& bpos) const {
  math::Matrix4d xform = m_rbtree->get_transform(tforms, m_index);
  if (bpos.isZero()) {
    return xform.tform2trvec();
  }
  return xform.tform2rotm() * bpos + xform.tform2trvec();
}

//==============================================================================
math::Quaternion RigidBody::get_xquat(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d xform = m_rbtree->get_transform(tforms, m_index);
  return xform.tform2quat();
}

//==============================================================================
math::Matrix3d RigidBody::get_xmat(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d xform = m_rbtree->get_transform(tforms, m_index);
  return xform.tform2rotm();
}

//==============================================================================
math::Matrix3d RigidBody::get_xmat(
    const std::vector<math::Matrix4d>& tforms,
    const math::Matrix3d& rotm) const {
  return get_xmat(tforms) * rotm;
}

//==============================================================================
math::Matrix4d RigidBody::get_xform(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix4d xform = m_rbtree->get_transform(tforms, m_index);
  return xform;
}

//==============================================================================
math::Vector6d RigidBody::get_xvel(
    const math::Jacobian& jacb, const math::VectorXd& qvel) const {
  return jacb * qvel;
}

//==============================================================================
math::Vector6d RigidBody::get_xacc(
    const math::Jacobian& jacb, const math::VectorXd& qacc) const {

  // missing J_dot * qvel term
  return jacb * qacc;
}

//==============================================================================
math::Vector6d RigidBody::get_xvel_at_bpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::VectorXd& qvel,
    const math::Vector3d& bpos) const {
  return get_jacb_at_bpos(tforms, jacb, bpos) * qvel;
}

//==============================================================================
math::Vector6d RigidBody::get_xacc_at_bpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::VectorXd& qacc,
    const math::Vector3d& bpos) const {

  // missing J_dot * qvel term
  return get_jacb_at_bpos(tforms, jacb, bpos) * qacc;
}

//==============================================================================
math::Jacobian RigidBody::get_jacb(
    const std::vector<math::Matrix4d>& tforms) const {
  return m_rbtree->get_body_jacb(tforms, m_index);
}

//==============================================================================
math::Jacobian RigidBody::get_jacb(
    const math::VectorXd& qpos,
    const std::vector<math::Matrix4d>& tforms,
    JacobianType type) const {
  return m_rbtree->get_body_jacb(qpos, tforms, m_index, type);
}

//==============================================================================
math::Jacobian RigidBody::get_jacb_at_bpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::Vector3d& bpos) const {
  // w' = w         =   I  x w + 0      = [   I    0 ] [w]
  // v' = w x r + v = [-r] x w + I x v  = [ [-r]x  I ] [v]
  math::Matrix6d jtwist    = math::Matrix6d::Identity();
  math::Vector3d xvec      = get_xmat(tforms) * bpos;
  jtwist.block<3, 3>(3, 0) = -xvec.skew();
  return jtwist * jacb;
}

//==============================================================================
math::Jacobian RigidBody::get_jacb_at_xpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::Vector3d& xpos) const {
  // w' = w         =   I  x w + 0      = [   I    0 ] [w]
  // v' = w x r + v = [-r] x w + I x v  = [ [-r]x  I ] [v]
  math::Matrix6d jtwist    = math::Matrix6d::Identity();
  math::Vector3d xvec      = xpos - get_xpos(tforms);
  jtwist.block<3, 3>(3, 0) = -xvec.skew();
  return jtwist * jacb;
}

//==============================================================================
math::AngularJacobian RigidBody::get_jacr(const math::Jacobian& jacb) const {
  return jacb.topRows(3);
}

//==============================================================================
math::LinearJacobian RigidBody::get_bvec_jacb(
    const std::vector<math::Matrix4d>& tforms,
    const math::AngularJacobian& jacr,
    const math::Vector3d bvec) const {
  // nvec  = R  bvec
  // nvec' = R' bvec = [w]x R bvec = -[R bvec]x w = -[R bvec]x Jr qdot
  math::Vector3d xvec = get_xmat(tforms) * bvec;
  return -xvec.skew() * jacr;
}

//==============================================================================
math::LinearJacobian RigidBody::get_jacp(const math::Jacobian& jacb) const {
  return jacb.bottomRows(3);
}

//==============================================================================
ConstJointPtr RigidBody::get_joint() const {
  return m_joint;
}

//==============================================================================
ConstSitePtr RigidBody::get_site(const std::string& name) const {
  for (size_t i = 0; i < m_sites.size(); i++) {
    if (m_sites[i]->get_name() == name) {
      return m_sites[i];
    }
  }
  return nullptr;
}

//==============================================================================
void RigidBody::add_site(ConstSitePtr site) {
  CROBOT_ASSERT_ERROR(
      get_site(site->get_name()) == nullptr,
      "Site with name '%s' already exists.",
      site->get_name().c_str());
  m_sites.push_back(site);
}

//==============================================================================
void RigidBody::add_site(
    const std::string& site_name,
    Site::Type type,
    const math::Vector3d& bpos,
    const math::Quaternion& quat,
    const math::Vector3d& size) {
  CROBOT_ASSERT_ERROR(
      get_site(site_name) == nullptr,
      "Site with name '%s' already exists.",
      site_name.c_str());
  m_sites.push_back(Site::MakeShared(
      site_name,
      type,
      std::dynamic_pointer_cast<RigidBody>(shared_from_this()),
      bpos,
      quat,
      size));
}

//==============================================================================
math::LinearJacobian RigidBody::get_jacp_at_bpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::Vector3d& bpos) const {
  math::Matrix36 jtwist = math::Matrix36::Zero();
  math::Vector3d xvec   = get_xmat(tforms) * bpos;
  //                                                   [w]
  // v' = w x r + v = [-r] x w + I x v  = [ [-r]x  I ] [v]
  jtwist.leftCols(3)  = -xvec.skew();
  jtwist.rightCols(3) = math::Matrix3d::Identity();
  return jtwist * jacb;
}

//==============================================================================
math::LinearJacobian RigidBody::get_jacp_at_xpos(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::Vector3d& xpos) const {
  math::Matrix36 jtwist = math::Matrix36::Zero();
  math::Vector3d xvec   = xpos - get_xpos(tforms);
  //                                                   [w]
  // v' = w x r + v = [-r] x w + I x v  = [ [-r]x  I ] [v]
  jtwist.leftCols(3)  = -xvec.skew();
  jtwist.rightCols(3) = math::Matrix3d::Identity();
  return jtwist * jacb;
}

//==============================================================================
math::Matrix6d RigidBody::spatial_inertia(
    double mass, const math::Vector3d& com, const math::Matrix3d& inertia) {
  math::Matrix3d sc = com.skew();
  math::Matrix6d spatialI;
  spatialI.block<3, 3>(0, 0) = inertia;
  spatialI.block<3, 3>(0, 3) = mass * sc;
  spatialI.block<3, 3>(3, 0) = mass * sc.transpose();
  spatialI.block<3, 3>(3, 3) = mass * math::Matrix3d::Identity();

  return spatialI;
}

//==============================================================================
math::Matrix3d RigidBody::transform_inertia_com2target(
    const math::Matrix3d& inertia_com,
    double mass,
    const math::Matrix3d& rotm,
    const math::Vector3d& p) {
  math::Matrix3d sp = p.skew();
  return rotm * inertia_com * rotm.transpose() + mass * sp * sp.transpose();
}

//==============================================================================
math::Matrix3d RigidBody::transform_inertia_body2com(
    const math::Matrix3d& inertia,
    double mass,
    const math::Matrix3d& rotm,
    const math::Vector3d& p) {
  math::Matrix3d sp = p.skew();
  return rotm.transpose() * (inertia - mass * sp * sp.transpose()) * rotm;
}

//==============================================================================
math::Vector6d RigidBody::get_ext_bfrc_btorq_with_bacc(
    const std::vector<math::Matrix4d>& tforms,
    const math::Jacobian& jacb,
    const math::VectorXd& qvel,
    const math::Vector3d baccp,
    const math::Vector3d baccr) const {

  math::Vector6d xvel  = get_xvel(jacb, qvel);
  math::Vector3d xvelr = xvel.head<3>();
  math::Matrix3d xmat  = get_xmat(tforms);
  math::Vector3d bvelr = xmat.transpose() * xvelr;

  math::Vector6d bfrc_btorq
      = inverse_dynamics_body_stationary_frame(bvelr, baccp, baccr);

  return bfrc_btorq;
}

//==============================================================================
math::Matrix6d RigidBody::get_spatial_inertia_body_stationary_frame() const {

  // Center of mass in the body stationary frame (instantaneously located at the
  // body frame and oriented the same way).
  math::Matrix3d s_bcom  = m_com.skew();
  math::Matrix3d s_bcom2 = s_bcom * s_bcom;

  // Spatial inertia matrix in the body stationary frame.
  math::Matrix6d spatialI = math::Matrix6d::Zero();

  // Inertia matrix represented in the inertial frame (located at the center of
  // mass and oriented the same as the body frame).
  math::Matrix3d I_cm = get_inertia_matrix_at_com();

  spatialI.block<3, 3>(0, 0) = m_mass * math::Matrix3d::Identity();
  spatialI.block<3, 3>(0, 3) = -m_mass * s_bcom;
  spatialI.block<3, 3>(3, 0) = m_mass * s_bcom;
  spatialI.block<3, 3>(3, 3) = I_cm - m_mass * s_bcom2;

  return spatialI;
}

//==============================================================================
math::Vector6d RigidBody::get_fictitious_bf_btau(
    const math::Vector3d& bvelr) const {
  math::Matrix3d s_bcom  = m_com.skew();
  math::Matrix3d s_bcom2 = s_bcom * s_bcom;
  math::Matrix3d I_cm    = get_inertia_matrix_at_com();
  math::Matrix3d s_bvelr = bvelr.skew(); // [omega_dot]x

  math::Vector6d fictF;
  fictF.segment(0, 3) = m_mass * s_bvelr * s_bvelr * m_com;
  fictF.segment(3, 3) = s_bvelr * (I_cm - m_mass * s_bcom2) * bvelr;

  return fictF;
}
