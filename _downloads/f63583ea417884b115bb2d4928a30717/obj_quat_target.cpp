#include "crobot/ik/obj_quat_target.hpp"

#include "crobot/common/common.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
ObjQuatTarget::ObjQuatTarget(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_name,
    const math::Quaternion& quat_target,
    double weight)
  : ObjTarget(rbtree, weight),
    m_body(rbtree->get_body(body_name)),
    m_quat_target(quat_target) {
}

//==============================================================================
math::VectorXd ObjQuatTarget::get_partial_deriv(
    const math::VectorXd& qpos, const math::VectorXd& qvel) const {
  (void)qvel;
  CROBOT_ASSERT_ERROR(qpos.size() == m_rbtree->get_qpos_dim());

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);
  math::Quaternion quat_body         = m_body->get_xquat(tforms);
  math::Vector4d quat_body_vec       = quat_body.as_vector();
  math::Vector4d quat_target_vec     = m_quat_target.as_vector();

  double dot_prod = quat_target_vec.transpose() * quat_body_vec;
  double part1    = -1.0 / (std::sqrt(1.0 - dot_prod * dot_prod));
  double part2    = dot_prod / (std::abs(dot_prod) + 1e-8);

  double q0 = quat_body.w();
  double q1 = quat_body.x();
  double q2 = quat_body.y();
  double q3 = quat_body.z();

  math::Matrix34 E;
  E.block<3, 1>(0, 0) = math::Vector3d(-q1, -q2, -q3);
  E.block<3, 1>(0, 1) = math::Vector3d(q0, q3, -q2);
  E.block<3, 1>(0, 2) = math::Vector3d(-q3, q0, q1);
  E.block<3, 1>(0, 3) = math::Vector3d(q2, -q1, q0);

  math::MatrixXd jacr = m_body->get_jacb(tforms).topRows(3);

  math::VectorXd deriv = (part1 * part2 * 0.5 * quat_target_vec.transpose()
                          * E.transpose() * jacr)
                             .transpose();

  return get_weight() * deriv;
}
