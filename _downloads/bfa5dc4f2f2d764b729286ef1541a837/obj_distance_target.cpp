#include "crobot/ik/obj_distance_target.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
ObjDistanceTarget::ObjDistanceTarget(
    kdl::ConstRigidBodyTreePtr rbtree,
    const std::string& body_name1,
    const math::Vector3d& bpos1,
    const std::string& body_name2,
    const math::Vector3d& bpos2,
    double checking_threshold,
    double weight)
  : ObjTarget(rbtree, weight),
    m_body1(rbtree->get_body(body_name1)),
    m_body2(rbtree->get_body(body_name2)),
    m_bpos1(bpos1),
    m_bpos2(bpos2),
    m_threshold(checking_threshold) {
}

//==============================================================================
math::VectorXd ObjDistanceTarget::get_partial_deriv(
    const math::VectorXd& qpos, const math::VectorXd& qvel) const {
  (void)qvel;
  CROBOT_ASSERT_ERROR(qpos.size() == m_rbtree->get_qpos_dim());

  std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(qpos);

  math::Vector3d xpos1 = m_body1->get_xpos(tforms, m_bpos1);
  math::Vector3d xpos2 = m_body2->get_xpos(tforms, m_bpos2);

  // no need for this objective if the two points are far away
  double dist = (xpos1 - xpos2).norm();
  if (m_threshold >= 0 && dist > m_threshold) {
    return math::VectorXd::Zero(qpos.size());
  }

  // std::cout << "distance active for " << m_body2->get_name() << std::endl;

  math::Jacobian jacb1 = m_body1->get_jacb(tforms);
  math::Jacobian jacb2 = m_body2->get_jacb(tforms);

  math::Matrix3d xmat1 = m_body1->get_xmat(tforms);
  math::Matrix3d xmat2 = m_body2->get_xmat(tforms);

  math::MatrixXd part1 = m_body1->get_jacp(jacb1)
                         - (xmat1 * m_bpos1).skew() * m_body1->get_jacr(jacb1);
  math::MatrixXd part2 = m_body2->get_jacp(jacb2)
                         - (xmat2 * m_bpos2).skew() * m_body2->get_jacr(jacb2);

  // std::cout << "part1: " << std::endl << part1 << std::endl;
  // std::cout << "part2: " << std::endl << part2 << std::endl;

  math::MatrixXd jacb = (part1 - part2).transpose();

  // we want to increase the distance, so go in positive direction
  math::VectorXd deriv = 2 * jacb * (xpos1 - xpos2);

  double weight_scale = 1.0;
  if (m_threshold >= 0)
    weight_scale = (m_threshold - dist) / (0.1 * m_threshold);

  return weight_scale * get_weight() * deriv;
}
