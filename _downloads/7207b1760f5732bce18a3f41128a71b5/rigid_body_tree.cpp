#include <queue>

#include "crobot/common/assert.hpp"
#include "crobot/common/tabulate.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/random.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
RigidBodyTreePtr RigidBodyTree::MakeShared(const std::string& name) {
  std::shared_ptr<RigidBodyTree> rbtree
      = std::shared_ptr<RigidBodyTree>(new RigidBodyTree(name));

  RigidBodyPtr base = kdl::RigidBody::MakeShared("world");
  base->m_index     = 0;
  base->m_rbtree    = rbtree;
  base->m_parent    = base;
  rbtree->m_bodies.push_back(base);

  return rbtree;
}

//==============================================================================
RigidBodyTree::RigidBodyTree(const std::string& name)
  : m_name(name), m_gravity(math::Vector3d(0.0, 0.0, -9.8)) {
  m_bodies.reserve(100);
  m_position_dof_map.reserve(100);
  m_velocity_dof_map.reserve(100);

  m_position_dof_map.push_back(std::make_pair<>(0, -1));
  m_velocity_dof_map.push_back(std::make_pair<>(0, -1));
  m_position_number = 0;
  m_velocity_number = 0;
  m_actuator_number = 0;

  m_num_non_fixed_bodies = 0;
  m_num_bodies           = 1;
}

//==============================================================================
void RigidBodyTree::add_body(
    kdl::RigidBodyPtr body, const std::string& parent_name) {
  // Check body name collision
  int bid1 = find_body_index_by_name(body->m_name);
  CROBOT_ASSERT_ERROR(bid1 == -1, "Body name collision.");

  // Check if the designated parent exists in robot
  int pid = validate_input_body_name(parent_name);

  // Check joint name collision
  int bid2 = find_body_index_by_joint_name(body->m_joint->m_name);
  CROBOT_ASSERT_ERROR(bid2 == -1, "Joint name collision.");

  int index     = m_num_bodies;
  body->m_index = index;
  m_bodies.push_back(body);

  kdl::RigidBodyPtr parent = m_bodies[pid];
  body->m_parent           = parent;
  parent->m_children.push_back(body);

  // mark joint for InTree
  body->m_joint->m_in_tree = true;
  body->m_joint->m_qpos_addr
      = std::make_pair<>(m_position_number, body->m_joint->get_qpos_dim());
  body->m_joint->m_qvel_addr
      = std::make_pair<>(m_velocity_number, body->m_joint->get_qvel_dim());

  // set rbtree pointer inside the body
  CROBOT_ASSERT_ERROR(
      body->m_rbtree == nullptr,
      "This body \"%s\" has been associated with another rigidBodyTree \"%s\", "
      "and cannot be added to this rigidBodyTree \"%s\".",
      body->m_name.c_str(),
      body->m_rbtree->m_name.c_str(),
      m_name.c_str());
  body->m_rbtree = shared_from_this();
  body->m_children.clear();

  // update robot numbers
  m_num_bodies++;

  // number of non-fixed bodies
  if (body->m_joint->m_type == JointType::FIXED) {
    m_num_non_fixed_bodies++;
  }

  // DoF
  m_position_dof_map.push_back(body->m_joint->m_qpos_addr);
  m_velocity_dof_map.push_back(body->m_joint->m_qvel_addr);
  m_position_number += body->m_joint->get_qpos_dim();
  m_velocity_number += body->m_joint->get_qvel_dim();
}

//==============================================================================
kdl::ConstRigidBodyPtr RigidBodyTree::get_body(const std::string& name) const {
  int bid = validate_input_body_name(name);
  return get_body(bid);
}

//==============================================================================
kdl::ConstRigidBodyPtr RigidBodyTree::get_body_by_joint(
    const std::string& name) const {
  int bid = find_body_index_by_joint_name(name);
  CROBOT_ASSERT_ERROR(bid > 0, "Invalid joint name: %s", name.c_str());
  return get_body(bid);
}

//==============================================================================
kdl::RigidBodyTreePtr RigidBodyTree::get_subtree(
    const std::string& subtree_name, const std::string& bodyname) const {
  int bid                  = validate_input_body_name(bodyname);
  RigidBodyTreePtr subtree = RigidBodyTree::MakeShared(subtree_name);
  std::queue<std::pair<RigidBodyPtr, std::string>> Q;
  Q.push(std::make_pair(get_body(bid), "world"));
  while (!Q.empty()) {
    std::pair<RigidBodyPtr, std::string> pair = Q.front();
    Q.pop();
    RigidBodyPtr body       = pair.first;
    std::string parent_name = pair.second;
    subtree->add_body(std::make_shared<RigidBody>(*body), parent_name);
    for (std::size_t i = 0; i < body->m_children.size(); i++) {
      Q.push(std::make_pair(body->m_children[i], body->m_name));
    }
  }
  return subtree;
}

//==============================================================================
kdl::RigidBodyPtr RigidBodyTree::get_body(int bid) const {
  CROBOT_ASSERT_ERROR(
      bid >= 0 && bid < m_num_bodies, "Invalid body id = %d", bid);
  if (bid < 0 || bid >= m_num_bodies) {
    return nullptr;
  }
  return m_bodies[bid];
}

//==============================================================================
std::vector<kdl::ConstRigidBodyPtr> RigidBodyTree::get_bodies() const {
  std::vector<kdl::ConstRigidBodyPtr> bodies;
  for (size_t i = 0; i < m_bodies.size(); i++) {
    bodies.push_back(m_bodies[i]);
  }
  return bodies;
}

//==============================================================================
kdl::ConstJointPtr RigidBodyTree::get_joint(const std::string& name) const {
  return get_body_by_joint(name)->get_joint();
}

//==============================================================================
int RigidBodyTree::get_num_bodies() const {
  return m_num_bodies;
}

//==============================================================================
int RigidBodyTree::get_qpos_dim() const {
  return m_position_number;
}

//==============================================================================
std::pair<int, int> RigidBodyTree::get_qpos_addr(
    const std::string& joint_name) const {
  ConstJointPtr joint = get_joint(joint_name);
  CROBOT_ASSERT_ERROR(joint != nullptr, "Invalid joint name.");
  CROBOT_ASSERT_ERROR(joint->m_type != JointType::FIXED, "Invalid joint type.");

  return joint->get_qpos_addr()[0];
}

//==============================================================================
std::vector<int> RigidBodyTree::get_qpos_addrs(
    const std::vector<std::string>& joint_names) const {
  std::vector<int> qpos_addrs;
  for (const std::string& joint_name : joint_names) {
    std::pair<int, int> qpos_addr = get_qpos_addr(joint_name);
    for (int i = qpos_addr.first; i < qpos_addr.first + qpos_addr.second; i++) {
      qpos_addrs.push_back(i);
    }
  }

  return qpos_addrs;
}

//==============================================================================
int RigidBodyTree::get_qvel_dim() const {
  return m_velocity_number;
}

//==============================================================================
std::pair<int, int> RigidBodyTree::get_qvel_addr(
    const std::string& joint_name) const {
  ConstJointPtr joint = get_joint(joint_name);
  CROBOT_ASSERT_ERROR(joint != nullptr, "Invalid joint name.");
  CROBOT_ASSERT_ERROR(joint->m_type != JointType::FIXED, "Invalid joint type.");

  return joint->get_qvel_addr()[0];
}

//==============================================================================
std::vector<int> RigidBodyTree::get_qvel_addrs(
    const std::vector<std::string>& joint_names) const {
  std::vector<int> qvel_addrs;
  for (const std::string& joint_name : joint_names) {
    std::pair<int, int> qvel_addr = get_qvel_addr(joint_name);
    for (int i = qvel_addr.first; i < qvel_addr.first + qvel_addr.second; i++) {
      qvel_addrs.push_back(i);
    }
  }

  return qvel_addrs;
}

//==============================================================================
int RigidBodyTree::get_ctrl_dim() const {
  return m_actuator_number;
}

//==============================================================================
math::VectorXd RigidBodyTree::subtract_qpos(
    math::VectorXd& qpos_a, math::VectorXd& qpos_b) const {
  math::VectorXd qvel(m_velocity_number);
  for (int i = 1; i < m_num_bodies; i++) {
    std::pair<int, int> qpos_addr = m_position_dof_map[i];
    std::pair<int, int> qvel_addr = m_velocity_dof_map[i];

    if (m_bodies[i]->m_joint->m_type == JointType::FIXED) {
      continue;
    }

    if (m_bodies[i]->m_joint->m_type == JointType::BALL) {
      math::VectorXd qpos_ai
          = qpos_a.segment(qpos_addr.first, qpos_addr.second);
      math::VectorXd qpos_bi
          = qpos_b.segment(qpos_addr.first, qpos_addr.second);
      math::Quaternion quat_ai(qpos_ai(0), qpos_ai(1), qpos_ai(2), qpos_ai(3));
      math::Quaternion quat_bi(qpos_bi(0), qpos_bi(1), qpos_bi(2), qpos_bi(3));
      qvel.segment(qvel_addr.first, qvel_addr.second)
          = quat_ai.subtract(quat_bi);
    }
    else if (
        m_bodies[i]->m_joint->m_type == JointType::PRISMATIC
        || m_bodies[i]->m_joint->m_type == JointType::REVOLUTE) {
      math::VectorXd qpos_ai
          = qpos_a.segment(qpos_addr.first, qpos_addr.second);
      math::VectorXd qpos_bi
          = qpos_b.segment(qpos_addr.first, qpos_addr.second);
      qvel.segment(qvel_addr.first, qvel_addr.second) = qpos_ai - qpos_bi;
    }
  }
  return qvel;
}

//==============================================================================
ConstCouplingPtr RigidBodyTree::add_joint_coupling(
    const std::string& name,
    const std::string& joint1,
    const std::string& joint2,
    const math::VectorXd& poly) {
  int bid1 = find_body_index_by_joint_name(joint1);
  int bid2 = find_body_index_by_joint_name(joint2);

  CouplingPtr cp = std::make_shared<Coupling>(
      name, m_bodies[bid1]->m_joint, m_bodies[bid2]->m_joint, poly);
  m_couplings.push_back(cp);
  return cp;
}

//==============================================================================
ConstTendonPtr RigidBodyTree::add_joint_tendon(
    const std::string& name,
    const std::vector<std::string>& joints,
    std::vector<double> coeffs) {
  CROBOT_ASSERT_ERROR(coeffs.size() == joints.size());
  CROBOT_ASSERT_ERROR(
      find_body_index_by_joint_name(name) < 0,
      "tendon name conflicts with one joint name.");

  TendonPtr tendon = std::make_shared<Tendon>(shared_from_this(), name);
  for (size_t i = 0; i < joints.size(); i++) {
    int bid = find_body_index_by_joint_name(joints[i]);
    CROBOT_ASSERT_ERROR(
        bid > 0, "Unable to find the joint name \"%s\"", joints[i].c_str());

    const JointPtr& joint = m_bodies[bid]->m_joint;
    tendon->add_joint(joint, coeffs[i]);
  }
  m_tendons.push_back(tendon);
  return tendon;
}

//==============================================================================
ConstActuatorPtr RigidBodyTree::add_joint_actuator(
    const std::string& name, const std::string& joint_name) {
  int bid = find_body_index_by_joint_name(joint_name);
  CROBOT_ASSERT_ERROR(
      bid > 0, "Unable to find the joint name \"%s\"", joint_name.c_str());
  ActuatorPtr actuator
      = std::make_shared<Actuator>(name, m_bodies[bid]->m_joint);
  m_actuators.push_back(actuator);
  m_actuator_number++;
  return actuator;
}

//==============================================================================
ConstActuatorPtr RigidBodyTree::add_tendon_actuator(
    const std::string& name, const std::string& tendon) {
  for (size_t i = 0; i < m_tendons.size(); i++) {
    if (m_tendons[i]->m_name == tendon) {
      ActuatorPtr actuator = std::make_shared<Actuator>(name, m_tendons[i]);
      m_actuators.push_back(actuator);
      m_actuator_number++;
      return actuator;
    }
  }
  return nullptr;
}

//==============================================================================
math::MatrixXd RigidBodyTree::get_tmat_qvel_coupling(
    const math::VectorXd& qpos) const {
  int num_couplings   = static_cast<int>(m_couplings.size());
  math::MatrixXd tmat = math::MatrixXd::Zero(num_couplings, m_velocity_number);

  for (int i = 0; i < num_couplings; i++) {
    const CouplingPtr& coupling    = m_couplings[i];
    std::pair<int, int> qpos_addr2 = coupling->m_joint2->m_qpos_addr;
    CROBOT_ASSERT_ERROR(qpos_addr2.second = 1);
    double q2                        = qpos(qpos_addr2.first);
    std::pair<double, double> coeffs = coupling->get_qvel_coeffs(q2);

    int qvel_addr1      = coupling->m_joint1->m_qvel_addr.first;
    int qvel_addr2      = coupling->m_joint2->m_qvel_addr.first;
    tmat(i, qvel_addr1) = coeffs.first;
    tmat(i, qvel_addr2) = coeffs.second;
  }

  return tmat;
}

//==============================================================================
math::MatrixXd RigidBodyTree::get_tmat_uvel2qvel(
    const math::VectorXd& qpos) const {
  math::MatrixXd C = get_tmat_qvel_coupling(qpos);
  math::MatrixXd U = get_tmat_qvel2uvel(qpos);
  CROBOT_ASSERT_ERROR(
      C.rows() + U.rows() == m_velocity_number,
      "C.rows() = %ld, U.rows() = %ld, m_velocity_number = %d",
      C.rows(),
      U.rows(),
      m_velocity_number);

  Eigen::FullPivLU<math::MatrixXd> lu(C);
  math::MatrixXd N = lu.kernel();

  return N * (U * N).inverse();
}

//==============================================================================
math::MatrixXd RigidBodyTree::get_tmat_qvel2uvel(
    const math::VectorXd& qpos) const {
  (void)qpos;
  math::MatrixXd tmat
      = math::MatrixXd::Zero(m_actuator_number, m_velocity_number);

  for (size_t i = 0; i < m_actuators.size(); i++) {
    const ActuatorPtr& actuator = m_actuators[i];
    std::vector<double> coeffs
        = actuator->m_transmission->get_transmission_coeffs();
    std::vector<std::pair<int, int>> qvel_addrs
        = actuator->m_transmission->get_qvel_addr();
    CROBOT_ASSERT_ERROR(coeffs.size() == qvel_addrs.size());
    for (size_t j = 0; j < coeffs.size(); j++) {
      std::pair<int, int> qvel_addr = qvel_addrs[j];
      CROBOT_ASSERT_ERROR(qvel_addr.second == 1);
      tmat(i, qvel_addr.first) = coeffs[j];
    }
  }

  return tmat;
}

//==============================================================================
std::vector<math::Matrix4d> RigidBodyTree::forward_kinematics(
    const math::VectorXd& qpos) const {
  int n = m_num_bodies;
  std::vector<math::Matrix4d> Ttree(n, math::Matrix4d::Identity());

  int k = 0;
  for (int i = 1; i < n; i++) {
    kdl::RigidBodyPtr body = m_bodies[i];
    int pnum               = body->m_joint->get_qpos_dim();
    math::VectorXd qi      = qpos.segment(k, pnum);
    Ttree[i]               = body->m_joint->tform_body2parent(qi);
    k                      = k + pnum;
    int pid                = m_bodies[i]->m_parent->m_index;
    if (pid > 0) {
      Ttree[i] = Ttree[pid] * Ttree[i];
    }
  }

  return Ttree;
}

//==============================================================================
math::Matrix4d RigidBodyTree::get_transform(
    const math::VectorXd& qpos,
    const std::string& body1,
    const std::string& body2) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  int bid1 = validate_input_body_name(body1);
  int bid2 = validate_input_body_name(body2);
  return get_transform(qpos, bid1, bid2);
}

//==============================================================================
math::Matrix4d RigidBodyTree::get_transform(
    const math::VectorXd& qpos, const std::string& body) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  int bid = validate_input_body_name(body);
  return get_transform(qpos, bid);
}

//==============================================================================
math::Jacobian RigidBodyTree::get_body_jacb(
    const math::VectorXd& qpos,
    const std::string& bodyname,
    JacobianType type) const {
  int bid = validate_input_body_name(bodyname);
  return get_body_jacb(qpos, bid, type);
}

//==============================================================================
math::Jacobian RigidBodyTree::get_body_jacb(
    const std::vector<math::Matrix4d>& tforms, const std::string& name) const {
  int bid = validate_input_body_name(name);
  return get_body_jacb(tforms, bid);
}

//==============================================================================
math::Jacobian RigidBodyTree::get_body_jacb(
    const std::vector<math::Matrix4d>& tforms, int bid) const {
  return get_body_jacb(
      math::VectorXd(), tforms, bid, JacobianType::WITH_RESPECT_TO_QVEL);
}

//==============================================================================
math::Jacobian RigidBodyTree::get_body_jacb(
    const math::VectorXd& qpos, int bid, JacobianType type) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  std::vector<math::Matrix4d> tforms = forward_kinematics(qpos);
  return get_body_jacb(qpos, tforms, bid, type);
}

//==============================================================================
math::Jacobian RigidBodyTree::get_body_jacb(
    const math::VectorXd& qpos,
    const std::vector<math::Matrix4d>& tforms,
    int bid,
    JacobianType type) const {
  CROBOT_ASSERT_ERROR(
      tforms.size() == m_bodies.size(),
      "tforms.size() = %ld, m_bodies.size() = %ld",
      tforms.size(),
      m_bodies.size());
  CROBOT_ASSERT_ERROR(
      type == JacobianType::WITH_RESPECT_TO_QVEL || //
      type == JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE);

  math::Jacobian jac = math::Jacobian::Zero(
      6,
      type == JacobianType::WITH_RESPECT_TO_QVEL ? m_velocity_number
                                                 : m_position_number);
  std::vector<int> chainmask(m_num_bodies, 0);

  math::Matrix4d T2, T2inv;
  if (bid == 0) {
    T2    = math::Matrix4d::Identity();
    T2inv = math::Matrix4d::Identity();
  }
  else {
    RigidBodyPtr body = m_bodies[bid];
    T2                = tforms[bid];
    T2inv             = T2.tform_inv();

    // mark the support set of end-effector
    chainmask[bid] = 1;
    int pid        = body->m_parent->m_index;
    while (pid != 0) {
      body           = m_bodies[pid];
      chainmask[pid] = 1;
      pid            = body->m_parent->m_index;
    }
  }

  for (int i = 1; i < m_num_bodies; i++) {
    RigidBodyPtr body = m_bodies[i];

    // only non-fixed bodies in the support set of end-effector
    if (body->m_joint->m_type != JointType::FIXED && chainmask[i]) {
      math::Matrix4d T1     = tforms[body->m_index];
      math::Matrix4d Tdh    = body->m_joint->m_tform_child2joint;
      math::Matrix4d Tdhinv = Tdh.tform_inv();
      math::Matrix4d T      = T2inv * T1 * Tdhinv;

      // WITH_RESPECT_TO_QVEL
      if (type == JacobianType::WITH_RESPECT_TO_QVEL) {
        std::pair<int, int> idx = m_velocity_dof_map[i];
        math::Jacobian JacSlice
            = T.tform2xform() * body->m_joint->m_motion_subspace;
        jac.middleCols(idx.first, idx.second) = JacSlice;
      }
      // WITH_RESPECT_TO_QPOS_DERIVATIVEP
      if (type == JacobianType::WITH_RESPECT_TO_QPOS_DERIVATIVE) {
        std::pair<int, int> idx = m_position_dof_map[i];

        math::Jacobian JacSlice;
        if (body->m_joint->m_type == JointType::BALL) {
          // quat = [w, x, y, z]
          math::Vector4d quat
              = qpos.segment(idx.first, idx.second).normalized();
          // use Quaternion element-wise constructor
          double q0 = quat(0);
          double q1 = quat(1);
          double q2 = quat(2);
          double q3 = quat(3);

          math::Matrix34 H;
          // clang-format off
          H << -q1,  q0,  q3, -q2, //
               -q2, -q3,  q0,  q1, //
               -q3,  q2, -q1,  q0; //
          // clang-format on
          JacSlice
              = T.tform2xform() * body->m_joint->m_motion_subspace * (2.0 * H);
        }
        else {
          JacSlice = T.tform2xform() * body->m_joint->m_motion_subspace;
        }

        jac.middleCols(idx.first, idx.second) = JacSlice;
      }
    }
  }
  math::Matrix6d X    = math::Matrix6d::Zero();
  X.block<3, 3>(0, 0) = T2.block<3, 3>(0, 0);
  X.block<3, 3>(3, 3) = T2.block<3, 3>(0, 0);

  jac = X * jac;

  // if (ball_qvel_start != -1) {
  //   jac.block<3, 3>(0, ball_qvel_start) = math::Matrix3d::Identity();
  // }

  return jac;
}

//==============================================================================
math::Vector3d RigidBodyTree::get_gravity_coeff() const {
  return m_gravity;
}

//==============================================================================
math::VectorXd RigidBodyTree::make_random_qpos() const {
  math::VectorXd qpos = math::VectorXd(m_position_number);
  for (int i = 1; i < m_num_bodies; i++) {
    JointPtr joint = m_bodies[i]->m_joint;
    if (joint->m_type != JointType::FIXED) {
      std::pair<int, int> idx             = m_position_dof_map[i];
      qpos.segment(idx.first, idx.second) = joint->random_qpos();
    }
  }
  return qpos;
}

//==============================================================================
math::VectorXd RigidBodyTree::make_random_qpos_offset(
    const math::VectorXd& qpos_ref,
    const math::VectorXd& qpos_offset_limit) const {
  math::VectorXd qpos = math::VectorXd(m_position_number);
  for (int i = 1; i < m_num_bodies; i++) {
    JointPtr joint = m_bodies[i]->m_joint;
    if (joint->m_type != JointType::FIXED) {
      std::pair<int, int> idx             = m_position_dof_map[i];
      qpos.segment(idx.first, idx.second) = joint->random_qpos_offset(
          qpos_ref(idx.first), qpos_offset_limit(idx.first));
    }
  }
  return qpos;
}

//==============================================================================
math::VectorXd RigidBodyTree::get_home_qpos() const {
  math::VectorXd qpos = math::VectorXd(m_position_number);
  for (int i = 1; i < m_num_bodies; i++) {
    JointPtr joint = m_bodies[i]->m_joint;
    if (joint->m_type != JointType::FIXED) {
      std::pair<int, int> idx             = m_position_dof_map[i];
      qpos.segment(idx.first, idx.second) = joint->m_home_position;
    }
  }
  return qpos;
}

//==============================================================================
math::VectorXd RigidBodyTree::make_zero_qpos() const {
  return math::VectorXd::Zero(m_position_number);
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> RigidBodyTree::get_qpos_limits()
    const {
  math::VectorXd qpos_lb(m_position_number);
  math::VectorXd qpos_ub(m_position_number);
  for (int i = 1; i < m_num_bodies; i++) {
    ConstRigidBodyPtr body = m_bodies[i];
    if (body->m_joint->m_type == JointType::FIXED) {
      continue;
    }
    std::pair<int, int> qpos_addr = m_position_dof_map[i];
    qpos_lb.segment(qpos_addr.first, qpos_addr.second)
        = body->m_joint->m_limits.first;
    qpos_ub.segment(qpos_addr.first, qpos_addr.second)
        = body->m_joint->m_limits.second;
  }
  return std::make_pair<>(qpos_lb, qpos_ub);
}

//==============================================================================
math::VectorXd RigidBodyTree::make_zero_qvel() const {
  return math::VectorXd::Zero(m_velocity_number);
}

//==============================================================================
math::VectorXd RigidBodyTree::make_zero_qacc() const {
  return math::VectorXd::Zero(m_velocity_number);
}

//==============================================================================
math::VectorXd RigidBodyTree::make_random_qvel() const {
  math::VectorXd qvel_lb = math::VectorXd::Constant(m_velocity_number, -1);
  math::VectorXd qvel_ub = math::VectorXd::Constant(m_velocity_number, 1);
  math::VectorXd qvel    = math::Random::uniform(qvel_lb, qvel_ub);
  return qvel;
}

//==============================================================================
math::VectorXd RigidBodyTree::make_random_qfrc() const {
  math::VectorXd qfrc_lb = math::VectorXd::Constant(m_velocity_number, -1);
  math::VectorXd qfrc_ub = math::VectorXd::Constant(m_velocity_number, 1);
  math::VectorXd qfrc    = math::Random::uniform(qfrc_lb, qfrc_ub);
  return qfrc;
}

//==============================================================================
std::vector<std::string> RigidBodyTree::get_movable_joint_names() const {
  std::vector<std::string> names;
  for (int i = 1; i < m_num_bodies; i++) {
    ConstRigidBodyPtr body = m_bodies[i];
    if (body->m_joint->m_type != JointType::FIXED) {
      names.push_back(body->m_joint->m_name);
    }
  }
  return names;
}

//==============================================================================
void RigidBodyTree::show_details() const {
  static std::unordered_map<JointType, std::string> jnt_type2str{
      {JointType::FIXED, "Fixed"},
      {JointType::REVOLUTE, "Revolute"},
      {JointType::PRISMATIC, "Prismatic"},
      {JointType::BALL, "Ball"},
  };
  tabulate::Table table;
  table.add_row(
      {"Body Id",
       "Body Name",
       "Joint Name (Idx)",
       "Joint Type",
       "Parent Name (Idx)",
       "Children Name(s)"});
  for (const RigidBodyPtr& body : std::as_const(m_bodies)) {
    // skip the base
    if (body->m_index == 0) {
      continue;
    }
    // joint information of the current body
    std::stringstream joint_names_ss;
    std::stringstream joint_types_ss;
    const JointPtr joint = body->m_joint;
    joint_names_ss << joint->m_name << "\n";
    joint_types_ss << jnt_type2str[joint->m_type] << "\n";
    std::string joint_names
        = joint_names_ss.rdbuf()->in_avail() ? joint_names_ss.str() : "None";
    std::string joint_types
        = joint_types_ss.rdbuf()->in_avail() ? joint_types_ss.str() : "Fixed";
    std::string body_parent_name = body->m_parent->m_name + " ("
                                   + std::to_string(body->m_parent->m_index)
                                   + ")";

    const auto& body_children = body->m_children;
    std::stringstream body_child_names_ss;
    for (std::size_t i = 0; i < body_children.size(); ++i) {
      body_child_names_ss << body_children[i]->m_name;
      body_child_names_ss << " (" << std::to_string(body_children[i]->m_index)
                          << ")";
      if (i + 1 < body_children.size()) {
        body_child_names_ss << "\n";
      }
    }
    std::string body_child_names = body_child_names_ss.str();

    table.add_row({
        std::to_string(body->m_index), // body id
        body->m_name,                  // body name
        joint_names,                   // joint id
        joint_types,                   // joint type
        body_parent_name,              // body parent name
        body_child_names               // children name
    });
  }
  // print the middle rows (not top and bottom border)
  for (std::size_t i = 2; i < m_bodies.size() - 1; ++i) {
    table[i].format().border_bottom(" ").border_top(" ");
  }
  // print the last row (no top border) if exists
  if (m_bodies.size() != 1) {
    // the last row
    table[m_bodies.size() - 1].format().border_top(" ");
  }
  std::cout << table << std::endl;
}

//==============================================================================
math::Vector3d RigidBodyTree::get_com_info(
    const math::VectorXd& qpos, math::Jacobian& comJac) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  math::Vector3d xpos_com;
  math::Jacobian cmm;
  double mass;
  centroidal_momentum_matrix(qpos, xpos_com, mass, cmm);
  comJac = cmm.bottomRows(3) / mass;
  return xpos_com;
}

//==============================================================================
std::vector<math::Vector6d> RigidBodyTree::compose_external_force(
    const std::string& bodyname,
    const math::VectorXd& qpos,
    const math::Vector6d& wrench) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number, "Invalid qpos vector.");
  int bid = validate_input_body_name(bodyname);
  CROBOT_ASSERT_ERROR(bid > 0, "Cannot set external force to base.");
  std::vector<math::Vector6d> fext_(m_num_bodies, math::Vector6d::Zero());
  std::vector<math::Matrix4d> tforms = forward_kinematics(qpos);
  math::Matrix6d X                   = tforms[bid].tform_inv().tform2xform();
  fext_[bid]                         = X.transpose() * wrench;
  return fext_;
}

//==============================================================================
math::VectorXd RigidBodyTree::forward_dynamics(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::VectorXd& qfrc,
    const std::vector<math::Vector6d>& fext) const {
  std::vector<int> lambda;
  math::MatrixXd H = mass_matrix(qpos, lambda);
  int vNum         = m_velocity_number;
  math::VectorXd bias
      = inverse_dynamics(qpos, qvel, math::VectorXd::Zero(vNum), fext);
  math::VectorXd y = qfrc - bias;
  math::MatrixXd L = special_cholesky(H, lambda);

  // specialized in-situ backward substitution
  // L^{-T}
  for (int i = vNum - 1; i >= 0; i--) {
    y(i)  = y(i) / L(i, i);
    int j = lambda[i];
    while (j >= 0) {
      y(j) = y(j) - y(i) * L(i, j);
      j    = lambda[j];
    }
  }

  // L^{-1}
  for (int i = 0; i < vNum; i++) {
    int j = lambda[i];
    while (j >= 0) {
      y(i) = y(i) - L(i, j) * y(j);
      j    = lambda[j];
    }
    y(i) = y(i) / L(i, i);
  }

  return y;
}

//==============================================================================
math::VectorXd RigidBodyTree::gravity_torque(const math::VectorXd& qpos) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  return inverse_dynamics(
      qpos,
      math::VectorXd::Zero(m_velocity_number),
      math::VectorXd::Zero(m_velocity_number),
      std::vector<math::Vector6d>(m_num_bodies, math::Vector6d::Zero()));
}

//==============================================================================
math::VectorXd RigidBodyTree::inverse_dynamics(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::VectorXd& qacc) const {
  return inverse_dynamics(
      qpos,
      qvel,
      qacc,
      std::vector<math::Vector6d>(m_num_bodies, math::Vector6d::Zero()));
}

//==============================================================================
math::VectorXd RigidBodyTree::inverse_dynamics(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::VectorXd& qacc,
    const std::vector<math::Vector6d>& fext) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  CROBOT_ASSERT_ERROR(qvel.size() == m_velocity_number);
  CROBOT_ASSERT_ERROR(qacc.size() == m_velocity_number);
  CROBOT_ASSERT_ERROR(fext.size() == m_bodies.size());
  math::Vector6d a0;
  a0.segment(0, 3) = math::Vector3d::Zero();
  a0.segment(3, 3) = -m_gravity;
  int nb           = m_num_bodies;

  // spatial transform from body parent(i) to body i
  std::vector<math::Matrix6d> X(nb, math::Matrix6d::Identity());
  // spatial transform from body i to base
  std::vector<math::Matrix6d> Xtree(nb, math::Matrix6d::Identity());
  // spatial velocity across joint i, in frame i
  std::vector<math::Vector6d> vJ(nb, math::Vector6d::Zero());
  // spatial velocity of body i
  std::vector<math::Vector6d> vB(nb, math::Vector6d::Zero());
  // spatial acceleration of body i
  std::vector<math::Vector6d> aB(nb, math::Vector6d::Zero());
  // spatial force on body i
  std::vector<math::Vector6d> f(nb, math::Vector6d::Zero());
  math::VectorXd qfrc = math::VectorXd ::Zero(m_velocity_number);

  // outbound
  for (int i = 1; i < nb; i++) {
    math::Matrix6X S      = m_bodies[i]->m_joint->m_motion_subspace;
    std::pair<int, int> a = m_position_dof_map[i];
    std::pair<int, int> b = m_velocity_dof_map[i];

    math::Matrix6d XDHOffset = math::Matrix6d::Identity();
    math::Matrix4d T;
    if (m_bodies[i]->m_joint->m_type == JointType::FIXED) {
      T     = m_bodies[i]->m_joint->tform_body2parent();
      vJ[i] = math::Vector6d::Zero();
    }
    else {
      math::VectorXd qpos_i = qpos.segment(a.first, a.second);
      math::VectorXd qvel_i = qvel.segment(b.first, b.second);

      T = m_bodies[i]->m_joint->tform_body2parent(qpos_i);
      math::Matrix4d TDHOffset = m_bodies[i]->m_joint->m_tform_child2joint;
      math::Matrix6d XDHOffset = TDHOffset.tform_inv().tform2xform();
      vJ[i]                    = XDHOffset * S * qvel_i;
    }
    X[i] = T.tform_inv().tform2xform();

    int pid = m_bodies[i]->m_parent->m_index;

    // if parent is not base
    if (pid > 0) {
      vB[i] = vJ[i] + X[i] * vB[pid];
      if (m_bodies[i]->m_joint->m_type == JointType::FIXED) {
        aB[i] = X[i] * aB[pid] + vB[i].cross_motion(vJ[i]);
      }
      else {
        math::VectorXd qacc_i = qacc.segment(b.first, b.second);
        aB[i]                 = X[i] * aB[pid] + XDHOffset * S * qacc_i
                + vB[i].cross_motion(vJ[i]);
      }
      Xtree[i] = Xtree[pid] * T.tform2xform();
    }
    // if parent is base
    else {
      vB[i] = vJ[i];
      if (m_bodies[i]->m_joint->m_type == JointType::FIXED) {
        aB[i] = X[i] * a0;
      }
      else {
        math::VectorXd qacc_i = qacc.segment(b.first, b.second);
        aB[i]                 = X[i] * a0 + XDHOffset * S * qacc_i;
      }
      Xtree[i] = T.tform2xform();
    }
    math::Matrix6d I = m_bodies[i]->m_spatial_inertia;
    math::Vector6d h = I * vB[i];
    f[i] = I * aB[i] + vB[i].cross_force(h) - Xtree[i].transpose() * fext[i];
  }

  // inbound
  for (int i = nb - 1; i > 0; i--) {
    if (m_bodies[i]->m_joint->m_type != JointType::FIXED) {
      math::Matrix4d TDHOffset = m_bodies[i]->m_joint->m_tform_child2joint;
      math::Matrix6d XDHOffset = TDHOffset.tform_inv().tform2xform();
      math::Matrix6X S = XDHOffset * m_bodies[i]->m_joint->m_motion_subspace;
      math::VectorXd tau_i            = S.transpose() * f[i];
      std::pair<int, int> b           = m_velocity_dof_map[i];
      qfrc.segment(b.first, b.second) = tau_i;
    }
    int pid = m_bodies[i]->m_parent->m_index;
    if (pid > 0) {
      f[pid] = f[pid] + X[i].transpose() * f[i];
    }
  }

  return qfrc;
}

//==============================================================================
math::MatrixXd RigidBodyTree::mass_matrix(const math::VectorXd& qpos) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  int nb = m_num_bodies;
  std::vector<math::Matrix6d> Ic(nb); // composite-rigid-body inertia
  std::vector<math::Matrix6d> X(nb);  // spatial transform from parent to body i
  int vNum         = m_velocity_number;
  math::MatrixXd H = math::MatrixXd::Zero(vNum, vNum);

  // preparation
  for (int i = 1; i < m_num_bodies; i++) {
    Ic[i]                 = m_bodies[i]->m_spatial_inertia;
    std::pair<int, int> p = m_position_dof_map[i];
    math::Matrix4d T;
    if (p.second == 0) {
      T = m_bodies[i]->m_joint->tform_body2parent();
    }
    else {
      CROBOT_ASSERT_ERROR(p.second > 0);
      T = m_bodies[i]->m_joint->tform_body2parent(
          qpos.segment(p.first, p.second));
    }
    X[i] = T.tform_inv().tform2xform();
  }

  // main loop
  for (int i = nb - 1; i > 0; i--) {
    int pid               = m_bodies[i]->m_parent->m_index;
    std::pair<int, int> a = m_velocity_dof_map[i];
    if (pid > 0) {
      Ic[pid] = Ic[pid] + X[i].transpose() * Ic[i] * X[i];
    }

    if (a.second > 0) {
      // An offset is needed when ChildToJointTransform is not identity (i.e.
      // when rigid body tree is created from DH parameters)
      math::Matrix4d TiDHOffset = m_bodies[i]->m_joint->m_tform_child2joint;
      math::Matrix6d XiDHOffset = TiDHOffset.tform_inv().tform2xform();
      math::Matrix6X Si = XiDHOffset * m_bodies[i]->m_joint->m_motion_subspace;
      math::Matrix6X Fi = Ic[i] * Si;
      H.block(a.first, a.first, a.second, a.second) = Si.transpose() * Fi;

      Fi    = X[i].transpose() * Fi;
      int j = pid;
      while (j > 0) {
        math::Matrix4d TjDHOffset = m_bodies[j]->m_joint->m_tform_child2joint;
        math::Matrix6d XjDHOffset = TjDHOffset.tform_inv().tform2xform();
        math::Matrix6X Sj
            = XjDHOffset * m_bodies[j]->m_joint->m_motion_subspace;
        std::pair<int, int> b = m_velocity_dof_map[j];
        if (b.second > 0) {
          math::MatrixXd Hji                            = Sj.transpose() * Fi;
          H.block(b.first, a.first, b.second, a.second) = Hji;
          H.block(a.first, b.first, a.second, b.second) = Hji.transpose();
        }
        Fi = X[j].transpose() * Fi;
        j  = m_bodies[j]->m_parent->m_index;
      }
    }
  }

  return H;
}

//==============================================================================
math::VectorXd RigidBodyTree::velocity_product(
    const math::VectorXd& qpos, const math::VectorXd& qvel) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  CROBOT_ASSERT_ERROR(qvel.size() == m_velocity_number);
  if (qvel.isZero()) {
    return math::VectorXd::Zero(m_velocity_number);
  }
  math::VectorXd qvel_zero = math::VectorXd::Zero(m_velocity_number);
  math::VectorXd qacc_zero = math::VectorXd::Zero(m_velocity_number);
  std::vector<math::Vector6d> fext(m_num_bodies, math::Vector6d::Zero());
  return inverse_dynamics(qpos, qvel, qacc_zero, fext)
         - inverse_dynamics(qpos, qvel_zero, qacc_zero, fext);
}

//==============================================================================
math::VectorXd RigidBodyTree::center_of_mass(const math::VectorXd& qpos) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  math::Vector3d com;
  double mass;
  math::Matrix6X cmm;
  centroidal_momentum_matrix(qpos, com, mass, cmm);
  return com;
}

//==============================================================================
math::VectorXd RigidBodyTree::center_of_mass(
    const math::VectorXd& qpos, math::LinearJacobian& jacp) const {
  math::Vector3d com;
  double mass;
  math::Matrix6X cmm;
  centroidal_momentum_matrix(qpos, com, mass, cmm);
  jacp = cmm.bottomRows(3) / mass;
  return com;
}

//==============================================================================
math::MatrixXd RigidBodyTree::mass_matrix(
    const math::VectorXd& qpos, std::vector<int>& lambda) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  int nb = m_num_bodies;
  std::vector<math::Matrix6d> Ic(nb); // composite-rigid-body inertia
  std::vector<math::Matrix6d> X(nb);  // spatial transform from parent to body i
  int vNum         = m_velocity_number;
  math::MatrixXd H = math::MatrixXd::Zero(vNum, vNum);
  lambda.resize(vNum);
  std::vector<int> lambda_(nb, 0);

  // preparation
  for (int i = 1; i < m_num_bodies; i++) {
    Ic[i]                 = m_bodies[i]->m_spatial_inertia;
    std::pair<int, int> p = m_position_dof_map[i];
    math::Matrix4d T;
    if (p.second == 0) {
      T = m_bodies[i]->m_joint->tform_body2parent();
    }
    else {
      T = m_bodies[i]->m_joint->tform_body2parent(
          qpos.segment(p.first, p.second));
    }
    X[i] = T.tform_inv().tform2xform();
  }

  // main loop
  for (int i = nb - 1; i > 0; i--) {
    int pid               = m_bodies[i]->m_parent->m_index;
    std::pair<int, int> a = m_velocity_dof_map[i];
    if (pid > 0) {
      Ic[pid]    = Ic[pid] + X[i].transpose() * Ic[i] * X[i];
      lambda_[i] = pid;
      while (lambda_[i] > 0
             && m_bodies[lambda_[i]]->m_joint->m_type == JointType::FIXED) {
        lambda_[i] = m_bodies[lambda_[i]]->m_parent->m_index;
      }
    }

    if (a.second > 0) {
      // An offset is needed when ChildToJointTransform is not identity (i.e.
      // when rigid body tree is created from DH parameters)
      math::Matrix4d TiDHOffset = m_bodies[i]->m_joint->m_tform_child2joint;
      math::Matrix6d XiDHOffset = TiDHOffset.tform_inv().tform2xform();
      math::Matrix6X Si = XiDHOffset * m_bodies[i]->m_joint->m_motion_subspace;
      math::Matrix6X Fi = Ic[i] * Si;
      H.block(a.first, a.first, a.second, a.second) = Si.transpose() * Fi;

      Fi    = X[i].transpose() * Fi;
      int j = pid;
      while (j > 0) {
        math::Matrix4d TjDHOffset = m_bodies[j]->m_joint->m_tform_child2joint;
        math::Matrix6d XjDHOffset = TjDHOffset.tform_inv().tform2xform();
        math::Matrix6X Sj
            = XjDHOffset * m_bodies[j]->m_joint->m_motion_subspace;
        std::pair<int, int> b = m_velocity_dof_map[j];
        if (b.second > 0) {
          math::MatrixXd Hji                            = Sj.transpose() * Fi;
          H.block(b.first, a.first, b.second, a.second) = Hji;
          H.block(a.first, b.first, a.second, b.second) = Hji.transpose();
        }
        Fi = X[j].transpose() * Fi;
        j  = m_bodies[j]->m_parent->m_index;
      }
    }
  }

  // post-processing lambda vector
  // lambda_(i) is either the index of the first non-fixed parent of body i or
  // 0 (base)
  std::vector<bool> mask(m_num_bodies, false);
  for (int i = 1; i < m_num_bodies; i++) {
    mask[i] = m_velocity_dof_map[i].second <= 0 ? false : true;
  }
  for (int i = 1; i < m_num_bodies; i++) {
    std::pair<int, int> a = m_velocity_dof_map[i];
    // skip fixed body
    if (a.second <= 0) {
      continue;
    }
    for (int k = a.first; k < a.first + a.second; k++) {
      lambda[k] = k - 1;
    }
    if (lambda_[i] == 0) {
      lambda[a.first] = -1;
    }
    else {
      std::pair<int, int> b = m_velocity_dof_map[lambda_[i]];
      lambda[a.first]       = b.first + b.second - 1;
    }
  }

  return H;
}

//==============================================================================
int RigidBodyTree::find_body_index_by_name(const std::string& name) const {
  for (int i = 0; i < m_num_bodies; i++) {
    if (m_bodies[i]->m_name == name) {
      return i;
    }
  }
  // Not able to find a body with the given name.
  return -1;
}

//==============================================================================
std::vector<int> RigidBodyTree::get_support_qpos_addr(int bid) const {
  std::vector<int> qpos_addr_dep;
  std::vector<ConstRigidBodyPtr> bodies = get_support_movable_bodies(bid);

  for (std::size_t i = 0; i < bodies.size(); i++) {
    std::pair<int, int> qpos_addr = m_position_dof_map[bodies[i]->m_index];
    for (int j = 0; j < qpos_addr.second; ++j) {
      qpos_addr_dep.push_back(qpos_addr.first + j);
    }
  }

  return qpos_addr_dep;
}

//==============================================================================
std::vector<int> RigidBodyTree::get_support_qvel_addr(int bid) const {
  std::vector<int> qvel_addr_dep;
  std::vector<ConstRigidBodyPtr> bodies = get_support_movable_bodies(bid);

  for (std::size_t i = 0; i < bodies.size(); i++) {
    std::pair<int, int> qvel_addr = m_velocity_dof_map[bodies[i]->m_index];
    for (int j = 0; j < qvel_addr.second; ++j) {
      qvel_addr_dep.push_back(qvel_addr.first + j);
    }
  }

  return qvel_addr_dep;
}

//==============================================================================
std::vector<ConstRigidBodyPtr> RigidBodyTree::get_support_movable_bodies(
    int bid) const {
  std::vector<ConstRigidBodyPtr> bodies;
  ConstRigidBodyPtr body = m_bodies[bid];

  // back tracking
  while (true) {
    std::pair<int, int> qpos_addr = m_position_dof_map[body->m_index];
    if (qpos_addr.second > 0) {
      bodies.push_back(body);
    }
    if (body->m_parent->m_index == 0) {
      break;
    }
    body = body->m_parent;
  }

  // from the root body to the queried body
  std::reverse(bodies.begin(), bodies.end());
  return bodies;
}

//==============================================================================
int RigidBodyTree::validate_input_body_name(const std::string& name) const {
  int bid = find_body_index_by_name(name);
  CROBOT_ASSERT_ERROR(bid >= 0, "Body name %s not found.", name.c_str());
  return bid;
}

//==============================================================================
int RigidBodyTree::find_body_index_by_joint_name(
    const std::string& name) const {
  for (int i = 1; i < m_num_bodies; i++) {
    if (m_bodies[i]->m_joint->m_name == name) {
      return i;
    }
  }
  return -1;
}

//==============================================================================
math::Matrix4d RigidBodyTree::get_transform(
    const math::VectorXd& qpos, int bid1, int bid2) const {
  std::vector<math::Matrix4d> tforms = forward_kinematics(qpos);
  math::Matrix4d tform1;
  if (bid1 == 0) {
    tform1 = math::Matrix4d::Identity();
  }
  else {
    tform1 = tforms[bid1];
  }

  math::Matrix4d tform2;
  if (bid2 == 0) {
    tform2 = math::Matrix4d::Identity();
  }
  else {
    tform2 = tforms[bid2];
  }

  return tform2.tform_inv() * tform1;
}

//==============================================================================
math::Matrix4d RigidBodyTree::get_transform(
    const math::VectorXd& qpos, int bid) const {
  std::vector<math::Matrix4d> tforms = forward_kinematics(qpos);
  return get_transform(tforms, bid);
}

//==============================================================================
math::Matrix4d RigidBodyTree::get_transform(
    const std::vector<math::Matrix4d>& tforms, int bid) const {
  math::Matrix4d tform;
  if (bid == 0) {
    tform = math::Matrix4d::Identity();
  }
  else {
    tform = tforms[bid];
  }
  return tform;
}

//==============================================================================
void RigidBodyTree::composite_rigid_body_inertia(
    const math::VectorXd& qpos,
    std::vector<math::Matrix6d>& IcrbBodies,
    std::vector<math::Matrix6d>& X) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_position_number);
  IcrbBodies.resize(m_num_bodies);
  IcrbBodies[0] = math::Matrix6d::Zero();
  X.resize(m_num_bodies);
  X[0] = math::Matrix6d::Zero();

  // outbound
  for (int i = 1; i < m_num_bodies; i++) {
    IcrbBodies[i]         = m_bodies[i]->m_spatial_inertia;
    std::pair<int, int> p = m_position_dof_map[i];
    math::Matrix4d T;
    if (p.second <= 0) {
      T = m_bodies[i]->m_joint->tform_body2parent();
    }
    else {
      T = m_bodies[i]->m_joint->tform_body2parent(
          qpos.segment(p.first, p.second));
    }
    X[i] = T.tform_inv().tform2xform();
  }

  // inbound
  for (int i = m_num_bodies - 1; i > 0; i--) {
    int pid         = m_bodies[i]->m_parent->m_index;
    IcrbBodies[pid] = IcrbBodies[pid] + X[i].transpose() * IcrbBodies[i] * X[i];
  }
}

//==============================================================================
void RigidBodyTree::centroidal_momentum_matrix(
    const math::VectorXd& qpos,
    math::Vector3d& pCoM,
    double& total_mass,
    math::Matrix6X& cmm) const {
  std::vector<math::Matrix6d> IcrbBodies;
  std::vector<math::Matrix6d> X;
  composite_rigid_body_inertia(qpos, IcrbBodies, X);

  math::Matrix6d IcrbBase = IcrbBodies[0];
  total_mass              = IcrbBase(5, 5);
  math::Matrix3d pCoMSkew = IcrbBase.block<3, 3>(0, 3) / total_mass;
  pCoM                    = pCoMSkew.vee();
  math::Matrix6d XG       = math::Matrix6d::Identity();
  XG.block<3, 3>(3, 0)    = pCoMSkew;

  int nb = m_num_bodies;
  std::vector<math::Matrix6d> Xtree(nb, math::Matrix6d::Identity());
  int vNum = m_velocity_number;

  cmm.resize(6, vNum);
  for (int i = 1; i < nb; i++) {
    RigidBodyPtr body = m_bodies[i];
    int pid           = body->m_parent->m_index;
    if (pid == 0) {
      Xtree[i] = Xtree[i] * XG;
    }
    else {
      Xtree[i] = Xtree[i] * Xtree[pid];
    }
    std::pair<int, int> a = m_velocity_dof_map[i];
    if (a.second > 0) {
      math::Matrix6X S         = body->m_joint->m_motion_subspace;
      math::Matrix4d TDHOffset = body->m_joint->m_tform_child2joint;
      math::Matrix6d XDHOffset = TDHOffset.tform_inv().tform2xform();
      cmm.middleCols(a.first, a.second)
          = Xtree[i].transpose() * IcrbBodies[i] * XDHOffset * S;
    }
  }
}

//==============================================================================
math::MatrixXd RigidBodyTree::special_cholesky(
    const math::MatrixXd& inertia_matrix, const std::vector<int>& lambda) {
  CROBOT_ASSERT_ERROR(inertia_matrix.rows() == inertia_matrix.cols());
  math::MatrixXd H = inertia_matrix;
  int n            = H.rows();

  for (int i = n - 1; i >= 0; i--) {
    H(i, i) = std::sqrt(H(i, i));
    int k   = lambda[i];
    while (k >= 0) {
      H(i, k) = H(i, k) / H(i, i);
      k       = lambda[k];
    }

    k = lambda[i];
    while (k >= 0) {
      int j = k;
      while (j >= 0) {
        H(k, j) = H(k, j) - H(i, k) * H(i, j);
        j       = lambda[j];
      }
      k = lambda[k];
    }
  }
  return H.triangularView<Eigen::Lower>();
}
