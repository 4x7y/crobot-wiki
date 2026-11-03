#include "crobot/simulation/mj_rigid_body_tree.hpp"

#include <memory>
#include <queue>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/common/tabulate.hpp"
#include "crobot/math/matrix.hpp"

using namespace crobot;
using namespace simulation;

//==============================================================================
MJRBTreePtr MJRBTree::MakeShared(const mjModel* model, mjData* data) {
  std::string name = (!model || !model->names || std::strlen(model->names) == 0)
                         ? std::string("model")
                         : std::string(model->names);
  return MakeShared(name, model, data, "world");
}

//==============================================================================
MJRBTreePtr MJRBTree::MakeShared(
    const std::string& name,
    const mjModel* model,
    mjData* data,
    const std::string& root_name) {
  int mjbid_root = mj_name2id(model, mjOBJ_BODY, root_name.c_str());
  CROBOT_ASSERT_ERROR(
      mjbid_root != -1, "Unknown root body name = %s.", root_name.c_str());
  CROBOT_ASSERT_ERROR(
      mjbid_root == 0 || model->body_parentid[mjbid_root] == 0,
      "Neither root body nor it parent is the world frame.");

  MJRBTreePtr rbtree = std::make_shared<MJRBTree>(name, model, data);

  // reset the base body as a MJRigidBody instance
  MJRigidBodyPtr base = MJRigidBody::MakeShared(model, 0, nullptr);
  base->m_index       = 0;
  base->m_rbtree      = rbtree;
  base->m_parent      = base;
  rbtree->m_bodies.push_back(base);

  std::vector<std::pair<int, int>> stack;
  if (mjbid_root == 0) {
    for (int i = model->nbody - 1; i > 0; i--) {
      if (model->body_parentid[i] == mjbid_root) {
        stack.push_back(std::make_pair<>(mjbid_root, i));
      }
    }
  }
  else {
    stack.push_back(std::make_pair<>(0, mjbid_root));
  }

  while (!stack.empty()) {
    // pop out the current body's id and its parent body's id
    auto [mjpid, mjbid] = stack.back();
    stack.pop_back();
    // add children ids to the stack
    for (int i = model->nbody; i >= 0; i--) {
      if (model->body_parentid[i] == mjbid) {
        stack.push_back(std::make_pair<>(mjbid, i));
      }
    }
    // add a Mujoco body
    rbtree->add_body_mj(mjpid, mjbid);
  }

  for (int i = 0; i < model->nsite; i++) {
    rbtree->add_site_mj(i);
  }

  // tendons
  for (int i = 0; i < model->ntendon; i++) {
    int objid_first = model->tendon_adr[i];
    int num_object  = model->tendon_num[i];
    std::vector<int> joint_ids;
    std::vector<double> coeffs;
    bool flag_add = true;
    for (int i = 0; i < num_object; ++i) {
      int objid = objid_first + i;

      // cannot find the joint
      if (rbtree->m_joint_addr.find(objid) == rbtree->m_joint_addr.end()) {
        flag_add = false;
        break;
      }

      CROBOT_ASSERT_ERROR(
          model->wrap_type[objid] == mjWRAP_JOINT,
          "Only JOINT type is supported in tendon class.");
      joint_ids.push_back(model->wrap_objid[objid]);
      coeffs.push_back(model->wrap_prm[objid]);
    }

    // add the tendon if all the joints on the tendon are in the RigidBodyTree.
    if (flag_add) {
      rbtree->add_joint_tendon_mj(i, joint_ids, coeffs);
    }
  }

  // actuators
  for (int i = 0; i < model->nu; i++) {
    int trans_type   = model->actuator_trntype[i];
    int trans_id     = model->actuator_trnid[2 * i];
    int trans_id_alt = model->actuator_trnid[2 * i + 1];
    CROBOT_ASSERT_WARNING(
        trans_id_alt == -1,
        "The second transmission ID is not defined in MUJOCO. Its value should "
        "be -1 in MUJOCO version 2.1.1.");
    CROBOT_ASSERT_ERROR(
        trans_type == mjTRN_JOINT || trans_type == mjTRN_TENDON,
        "Unsupported transmission type (%d) for actuator.",
        trans_type);
    if (trans_type == mjTRN_JOINT &&                                      //
        rbtree->m_joint_addr.find(trans_id) != rbtree->m_joint_addr.end() //
    ) {
      rbtree->add_joint_actuator_mj(i, trans_id);
    }
    else if (
        trans_type == mjTRN_TENDON &&                                       //
        rbtree->m_tendon_addr.find(trans_id) != rbtree->m_tendon_addr.end() //
    ) {
      rbtree->add_tendon_actuator_mj(i, trans_id);
    }
  }

  // joint couplings
  for (int i = 0; i < model->neq; i++) {
    if (model->eq_type[i] != mjtEq::mjEQ_JOINT) {
      continue;
    }
    int jid1 = model->eq_obj1id[i];
    int jid2 = model->eq_obj2id[i];

    // add the equality constraint only if both joints belong to this robot
    if (rbtree->m_joint_addr.find(jid1) != rbtree->m_joint_addr.end() && //
        rbtree->m_joint_addr.find(jid2) != rbtree->m_joint_addr.end()    //
    ) {
      // clang-format off
      math::VectorXd coeffs = (math::VectorXd(5) << 
        model->eq_data[4],
        model->eq_data[3],
        model->eq_data[2],
        model->eq_data[1],
        model->eq_data[0]).finished().trim_leading_zeros();
      // clang-format on
      rbtree->add_joint_coupling(
          std::string(mj_id2name(model, mjOBJ_EQUALITY, i)),
          std::string(mj_id2name(model, mjOBJ_JOINT, jid1)),
          std::string(mj_id2name(model, mjOBJ_JOINT, jid2)),
          coeffs);
    }
    else {
      CROBOT_ASSERT_WARNING(
          !(rbtree->m_joint_addr.find(jid1) != rbtree->m_joint_addr.end() && //
            rbtree->m_joint_addr.find(jid2) != rbtree->m_joint_addr.end()),  //
          "Ignore the equality constraint as only only one joint belongs to "
          "this RigidBodyTree.");
    }
  }

  return rbtree;
}

//==============================================================================
MJRBTree::MJRBTree(const std::string& name, const mjModel* model, mjData* data)
  : kdl::RigidBodyTree(name), m_model(model), m_data(data) {
  CROBOT_ASSERT_ERROR(m_model != nullptr);
  CROBOT_ASSERT_ERROR(m_data != nullptr);

  // reset the gravitational acceleration according to Mujoco model file
  m_gravity = math::Vector3d::Map(model->opt.gravity);

  // set qpos_addr
  m_qpos_addr.first  = -1;
  m_qpos_addr.second = 0;

  // set qvel_addr
  m_qvel_addr.first  = -1;
  m_qvel_addr.second = 0;

  // set ctrl_addr
  m_ctrl_addr.first  = -1;
  m_ctrl_addr.second = 0;

  // set body_addr
  m_body_addr.insert(0);
}

//==============================================================================
ConstMJRigidBodyPtr MJRBTree::get_body(const std::string& name) const {
  return std::dynamic_pointer_cast<const MJRigidBody>(
      kdl::RigidBodyTree::get_body(name));
}

//==============================================================================
ConstMJRigidBodyPtr MJRBTree::get_body_by_joint(const std::string& name) const {
  return std::dynamic_pointer_cast<const MJRigidBody>(
      kdl::RigidBodyTree::get_body_by_joint(name));
}

//==============================================================================
std::vector<ConstMJRigidBodyPtr> MJRBTree::get_bodies() const {
  std::vector<ConstMJRigidBodyPtr> bodies;
  for (const auto& body : m_bodies) {
    bodies.push_back(std::dynamic_pointer_cast<const MJRigidBody>(body));
    CROBOT_ASSERT_ERROR(bodies.back() != nullptr);
  }
  return bodies;
}

//==============================================================================
std::vector<ConstMJRigidBodyPtr> MJRBTree::get_bodies_no_dummy() const {
  std::vector<ConstMJRigidBodyPtr> bodies;
  for (size_t i = 0; i < m_bodies.size(); i++) {
    ConstMJRigidBodyPtr body
        = std::dynamic_pointer_cast<const MJRigidBody>(m_bodies[i]);
    if (!body->is_dummy_body()) {
      bodies.push_back(body);
      CROBOT_ASSERT_ERROR(bodies.back() != nullptr);
    }
  }
  return bodies;
}

//==============================================================================
math::Matrix4d MJRBTree::get_transform_mj(const std::string& body) const {
  int mjbid = mj_name2id(m_model, mjOBJ_BODY, body.c_str());
  CROBOT_ASSERT_ERROR(mjbid != -1, "Unknown body name (%s).", body.c_str());
  return get_transform_mj(mjbid);
}

//==============================================================================
math::Matrix4d MJRBTree::get_transform_mj(
    const std::string& body1, const std::string& body2) const {
  int mjbid1 = mj_name2id(m_model, mjOBJ_BODY, body1.c_str());
  CROBOT_ASSERT_ERROR(mjbid1 != -1, "Unknown body name.");
  int mjbid2 = mj_name2id(m_model, mjOBJ_BODY, body2.c_str());
  CROBOT_ASSERT_ERROR(mjbid2 != -1, "Unknown body name.");
  return get_transform_mj(mjbid1, mjbid2);
}

//==============================================================================
void MJRBTree::set_qpos_mj(const math::VectorXd& qpos) const {
  CROBOT_ASSERT_ERROR(qpos.size() == m_qpos_addr.second);
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos;
}

//==============================================================================
void MJRBTree::set_qpos_mj(
    const std::string& joint_name, const math::VectorXd& qpos) const {
  ConstMJRigidBodyPtr body = get_body_by_joint(joint_name);
  CROBOT_ASSERT_ERROR(body != nullptr);
  std::pair<int, int> qpos_addr = body->get_qpos_addr();
  CROBOT_ASSERT_ERROR(qpos.size() == qpos_addr.second);
  for (int i = 0; i < qpos_addr.second; i++) {
    m_data->qpos[qpos_addr.first + i] = qpos(i);
  }
}

//==============================================================================
void MJRBTree::set_qpos_mj(const std::string& joint_name, double qpos) const {
  set_qpos_mj(joint_name, math::Vector1d(qpos));
}

//==============================================================================
void MJRBTree::set_qpos_by_body_mj(
    const std::string& bodyname, const math::VectorXd& qpos) const {
  ConstMJRigidBodyPtr body = get_body(bodyname);
  CROBOT_ASSERT_ERROR(body != nullptr);
  std::pair<int, int> qpos_addr = body->get_qpos_addr();
  CROBOT_ASSERT_ERROR(qpos.size() == qpos_addr.second);
  for (int i = 0; i < qpos_addr.second; i++) {
    m_data->qpos[qpos_addr.first + i] = qpos(i);
  }
}

//==============================================================================
math::VectorXd MJRBTree::get_qpos_mj() const {
  return math::VectorXd::Map(
      m_data->qpos + m_qpos_addr.first, m_qpos_addr.second);
}

//==============================================================================
math::VectorXd MJRBTree::get_qpos_mj(const std::string& joint_name) const {
  ConstMJRigidBodyPtr body = get_body_by_joint(joint_name);
  CROBOT_ASSERT_ERROR(body != nullptr);
  std::pair<int, int> qpos_addr = body->get_qpos_addr();
  CROBOT_ASSERT_ERROR(
      qpos_addr.second > 0, "Can not get qpos from a body with fixed joint.");
  math::VectorXd qpos(qpos_addr.second);
  for (int i = 0; i < qpos_addr.second; i++) {
    qpos(i) = m_data->qpos[qpos_addr.first + i];
  }
  return qpos;
}

//==============================================================================
math::VectorXd MJRBTree::get_qpos_by_body(const std::string& bodyname) const {
  ConstMJRigidBodyPtr body = get_body(bodyname);
  CROBOT_ASSERT_ERROR(body != nullptr);
  std::pair<int, int> qpos_addr = body->get_qpos_addr();
  CROBOT_ASSERT_ERROR(
      qpos_addr.second > 0, "Can not get qpos from a body with fixed joint.");
  math::VectorXd qpos(qpos_addr.second);
  for (int i = 0; i < qpos_addr.second; i++) {
    qpos(i) = m_data->qpos[qpos_addr.first + i];
  }
  return qpos;
}

//==============================================================================
void MJRBTree::set_qvel_mj(const math::VectorXd& qvel) const {
  CROBOT_ASSERT_ERROR(qvel.size() == m_qvel_addr.second);
  math::VectorXd::Map(m_data->qvel + m_qvel_addr.first, m_qvel_addr.second)
      = qvel;
}

//==============================================================================
void MJRBTree::set_qvel_mj(
    const std::string& joint_name, const math::VectorXd& qvel) const {

  ConstMJRigidBodyPtr body = get_body_by_joint(joint_name);
  CROBOT_ASSERT_ERROR(body != nullptr);
  std::pair<int, int> qvel_addr = body->get_qvel_addr();
  CROBOT_ASSERT_ERROR(qvel.size() == qvel_addr.second);
  for (int i = 0; i < qvel_addr.second; i++) {
    m_data->qpos[qvel_addr.first + i] = qvel(i);
  }
}

//==============================================================================
math::VectorXd MJRBTree::get_qvel_mj() const {
  return math::VectorXd::Map(
      m_data->qvel + m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
void MJRBTree::set_qacc_mj(const math::VectorXd& qacc) const {
  CROBOT_ASSERT_ERROR(qacc.size() == m_qvel_addr.second);
  math::VectorXd::Map(m_data->qacc + m_qvel_addr.first, m_qvel_addr.second)
      = qacc;
}

//==============================================================================
math::VectorXd MJRBTree::get_qacc_mj() const {
  return math::VectorXd::Map(
      m_data->qacc + m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
void MJRBTree::set_ctrl_mj(const math::VectorXd& ctrl) const {
  CROBOT_ASSERT_ERROR(
      ctrl.size() == m_ctrl_addr.second,
      "ctrl.size() = %ld, m_ctrl_addr.second = %d",
      ctrl.size(),
      m_ctrl_addr.second);
  math::VectorXd::Map(m_data->ctrl + m_ctrl_addr.first, m_ctrl_addr.second)
      = ctrl;
}

//==============================================================================
math::VectorXd MJRBTree::get_ctrl_mj() const {
  return math::VectorXd::Map(
      m_data->ctrl + m_ctrl_addr.first, m_ctrl_addr.second);
}

//==============================================================================
void MJRBTree::set_qfrc_applied_mj(const math::VectorXd& qfrc) const {
  CROBOT_ASSERT_ERROR(qfrc.size() == m_qvel_addr.second);
  math::VectorXd::Map(
      m_data->qfrc_applied + m_qvel_addr.first, m_qvel_addr.second)
      = qfrc;
}

//==============================================================================
math::VectorXd MJRBTree::get_qfrc_inverse_mj() const {
  return math::VectorXd::Map(
      m_data->qfrc_inverse + m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
void MJRBTree::set_external_wrench(
    const std::string& bodyname, const math::Vector6d& wrench) const {
  int bid = validate_input_body_name(bodyname);
  CROBOT_ASSERT_ERROR(bid != 0, "Cannot apply external wrench to base");
  ConstMJRigidBodyPtr body
      = std::dynamic_pointer_cast<const MJRigidBody>(m_bodies[bid]);
  CROBOT_ASSERT_ERROR(
      !body->is_dummy_body(),
      "Cannot apply external wrench to dummy body \"%s\" (id = %d). Skip.",
      body->get_name().c_str(),
      body->m_index);
  set_external_wrench(bid, wrench);
}

//==============================================================================
void MJRBTree::set_xtorq_xfrc(
    const std::string& bodyname,
    const math::Vector3d& xtorq,
    const math::Vector3d& xfrc) const {
  int bid = validate_input_body_name(bodyname);
  CROBOT_ASSERT_ERROR(
      bid != 0, "Cannot apply external force and torque to base");
  ConstMJRigidBodyPtr body
      = std::dynamic_pointer_cast<const MJRigidBody>(m_bodies[bid]);
  CROBOT_ASSERT_ERROR(
      !body->is_dummy_body(),
      "Cannot apply external force and torque to dummy body \"%s\" (id = %d). "
      "Skip.",
      body->get_name().c_str(),
      body->m_index);
  set_xtorq_xfrc(bid, xtorq, xfrc);
}

//==============================================================================
void MJRBTree::set_external_wrench(
    const std::vector<math::Vector6d>& wrench) const {
  CROBOT_ASSERT_ERROR(
      static_cast<int>(wrench.size()) == m_num_bodies,
      "The number of wrenches specified should be same as the number of bodies "
      "in this robot.");
  CROBOT_ASSERT_ERROR(wrench[0].isZero(), "Cannot set external force to base.");
  for (int bid = 1; bid < m_num_bodies; bid++) {
    ConstMJRigidBodyPtr body
        = std::dynamic_pointer_cast<const MJRigidBody>(m_bodies[bid]);
    if (body->is_dummy_body()) {
      CROBOT_ASSERT_ERROR(
          wrench[bid].isZero(),
          "Cannot apply external wrench to dummy body \"%s\" (id = %d).",
          body->get_name().c_str(),
          body->m_index);
      continue;
    }
    set_external_wrench(bid, wrench[bid]);
  }
}

//==============================================================================
void MJRBTree::forward_kinematics_mj() const {
  mj_kinematics(m_model, m_data);
}

//==============================================================================
math::Jacobian MJRBTree::get_body_jacb_mj(const std::string& bodyname) const {
  return get_body_jacb_mj(body_name2id(bodyname));
}

//==============================================================================
math::MatrixXd MJRBTree::mass_matrix_with_armature_inertia(
    const math::VectorXd& qpos) const {
  math::MatrixXd H = kdl::RigidBodyTree::mass_matrix(qpos);

  // Armature inertia (or rotor inertia, or reflected inertia) of all degrees of
  // freedom created by this joint. These are constants added to the diagonal of
  // the inertia matrix in generalized coordinates. They make the simulation
  // more stable, and often increase physical realism. This is because when a
  // motor is attached to the system with a transmission that amplifies the
  // motor force by c, the inertia of the rotor (i.e., the moving part of the
  // motor) is amplified by c*c. The same holds for gears in the early stages of
  // planetary gear boxes. These extra inertias often dominate the inertias of
  // the robot parts that are represented explicitly in the model, and the
  // armature attribute is the way to model them.
  H.diagonal()
      += math::VectorXd::Map(m_model->dof_armature, m_qvel_addr.second);
  return H;
}

//==============================================================================
math::MatrixXd MJRBTree::mass_matrix_mj() const {
  // Convert sparse inertia matrix qM into full (i.e. dense) matrix.
  math::MatrixXd mass_matrix(m_model->nv, m_model->nv);
  mj_fullM(m_model, mass_matrix.data(), m_data->qM);

  // Return the submatrix of inertia matrix that related to this RigidBodyTree.
  return mass_matrix.block(
      m_qvel_addr.first,
      m_qvel_addr.first,
      m_qvel_addr.second,
      m_qvel_addr.second);
}

//==============================================================================
math::Vector3d MJRBTree::center_of_mass_mj() const {
  return math::Vector3d::Map(m_data->subtree_com);
}

//==============================================================================
math::VectorXd MJRBTree::velocity_product_mj() const {
  math::VectorXd qfrc(m_model->nv);
  mj_rne(m_model, m_data, 0, qfrc.data());

  // back up the current qvel
  math::VectorXd qvel = get_qvel_mj();

  // compute the generalized force when qvel = 0
  set_qvel_mj(make_zero_qvel());
  mj_fwdVelocity(m_model, m_data);
  math::VectorXd qfrc_zerovel(m_model->nv);
  mj_rne(m_model, m_data, 0, qfrc_zerovel.data());

  // recover the original qvel
  set_qvel_mj(qvel);
  mj_fwdVelocity(m_model, m_data);

  return (qfrc - qfrc_zerovel).segment(m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
math::VectorXd MJRBTree::inverse_dynamics(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    const math::VectorXd& qacc,
    const std::vector<math::Vector6d>& wrench) const {
  CROBOT_ASSERT_ERROR(wrench.size() == m_bodies.size());
  CROBOT_ASSERT_ERROR(
      wrench[0].isZero(), "Cannot apply wrench to the base frame. Skip.");
  CROBOT_ASSERT_ERROR(
      std::find_if(
          m_bodies.begin() + 1,
          m_bodies.end(),
          [&wrench](const kdl::ConstRigidBodyPtr& body) {
            ConstMJRigidBodyPtr mjbody
                = std::dynamic_pointer_cast<const MJRigidBody>(body);
            return mjbody->is_dummy_body() && !wrench[mjbody->m_index].isZero();
          })
          == m_bodies.end(),
      "Cannot apply external wrench to dummy bodies. Skip.");
  math::VectorXd bias = kdl::RigidBodyTree::inverse_dynamics(
      qpos, qvel, make_zero_qacc(), wrench);
  math::MatrixXd M = kdl::RigidBodyTree::mass_matrix(qpos);
  return M * qacc + bias;
}

//==============================================================================
math::VectorXd MJRBTree::inverse_dynamics_mj() const {
  math::VectorXd qfrc_rne(m_model->nv);
  mj_rne(m_model, m_data, 1, qfrc_rne.data());

  for (int i = 1; i < m_num_bodies; i++) {
    MJRigidBodyPtr body = std::dynamic_pointer_cast<MJRigidBody>(m_bodies[i]);
    if (body->is_dummy_body()) {
      continue;
    }

    // force and torque applied to the robot body from the environment
    math::Vector3d xfrc
        = math::Vector3d::Map(m_data->xfrc_applied + body->m_mjid * 6 + 0);
    math::Vector3d xtrq
        = math::Vector3d::Map(m_data->xfrc_applied + body->m_mjid * 6 + 3);

    math::Matrix3X jacp(3, m_model->nv);
    math::Matrix3X jacr(3, m_model->nv);
    mj_jacBody(m_model, m_data, jacp.data(), jacr.data(), body->m_mjid);
    qfrc_rne -= jacp.transpose() * xfrc;
    qfrc_rne -= jacr.transpose() * xtrq;
  }

  return qfrc_rne.segment(m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
math::VectorXd MJRBTree::forward_dynamics_mj() const {
  mj_forward(m_model, m_data);
  return get_qacc_mj();
}

//==============================================================================
void MJRBTree::collision_checking_mj() const {
  mj_collision(m_model, m_data);
}

//==============================================================================
void MJRBTree::show_addrs() const {
  tabulate::Table table;
  table.add_row(
      {"Joint Name", "qpos Addr", "qvel Addr", "Actuator Name", "ctrl Addr"});
  std::vector<std::string> qpos_names;
  std::vector<std::string> qpos_addrs;
  std::vector<std::string> qvel_addrs;
  std::vector<std::string> ctrl_names;
  std::vector<std::string> ctrl_addrs;

  for (std::size_t i = 1; i < m_bodies.size(); i++) {
    MJRigidBodyPtr body = std::dynamic_pointer_cast<MJRigidBody>(m_bodies[i]);
    MJJointPtr joint    = std::dynamic_pointer_cast<MJJoint>(body->m_joint);
    auto qpos_addr      = joint->m_qpos_addr;
    qpos_names.push_back(joint->get_name());
    qpos_addrs.push_back(
        "[" + std::to_string(qpos_addr.first) + ":"
        + std::to_string(qpos_addr.second) + "]");
    auto qvel_addr = joint->m_qvel_addr;
    qvel_addrs.push_back(
        "[" + std::to_string(qvel_addr.first) + ":"
        + std::to_string(qvel_addr.second) + "]");
  }
  // for (const ConstActuatorPtr& actuator : std::as_const(m_actuators)) {
  //   ctrl_names.push_back(actuator->get_name());
  //   ctrl_addrs.push_back(std::to_string(actuator->get_ctrl_addr()));
  // }
  std::size_t num_rows = std::max(qpos_names.size(), ctrl_names.size());
  for (std::size_t i = 0; i < num_rows; ++i) {
    table.add_row({
        i < qpos_names.size() ? qpos_names[i] : "",
        i < qpos_addrs.size() ? qpos_addrs[i] : "",
        i < qvel_addrs.size() ? qvel_addrs[i] : "",
        i < ctrl_names.size() ? ctrl_names[i] : "",
        i < ctrl_addrs.size() ? ctrl_addrs[i] : "",
    });
  }
  for (std::size_t i = 2; i < num_rows; ++i) {
    table[i][0].format().border_bottom(" ").border_top(" ").border_right(" ");
    table[i][1]
        .format()
        .border_bottom(" ")
        .border_top(" ")
        .border_left(" ")
        .border_right(" ");
    table[i][2].format().border_bottom(" ").border_top(" ").border_left(" ");
    table[i][3].format().border_bottom(" ").border_top(" ").border_right(" ");
    table[i][4].format().border_bottom(" ").border_top(" ").border_left(" ");
  }

  if (num_rows > 1) {
    table[1][0].format().border_bottom(" ").border_right(" ");
    table[1][1].format().border_bottom(" ").border_left(" ").border_right(" ");
    table[1][2].format().border_bottom(" ").border_left(" ");
    table[1][3].format().border_bottom(" ").border_right(" ");
    table[1][4].format().border_bottom(" ").border_left(" ");

    table[num_rows][0].format().border_top(" ").border_right(" ");
    table[num_rows][1].format().border_top(" ").border_left(" ").border_right(
        " ");
    table[num_rows][2].format().border_top(" ").border_left(" ");
    table[num_rows][3].format().border_top(" ").border_right(" ");
    table[num_rows][4].format().border_top(" ").border_left(" ");
  }
  else if (num_rows > 0) {
    table[num_rows][0].format().border_right(" ");
    table[num_rows][1].format().border_left(" ").border_right(" ");
    table[num_rows][2].format().border_left(" ");
    table[num_rows][3].format().border_right(" ");
    table[num_rows][4].format().border_left(" ");
  }
  std::cout << table << std::endl;
}

//==============================================================================
const mjModel* MJRBTree::get_model_mj() const {
  return m_model;
}

//==============================================================================
mjData* MJRBTree::get_data_mj() const {
  return m_data;
}

//==============================================================================
bool MJRBTree::check_collision() const {
  mj_collision(m_model, m_data);
  return m_data->ncon > 0;
}

//==============================================================================
std::vector<kdl::BodyContact::ConstPtr> MJRBTree::get_contacts() const {
  std::vector<kdl::BodyContact::ConstPtr> contacts;
  contacts.reserve(m_data->ncon);
  for (int i = 0; i < m_data->ncon; i++) {
    mjContact* con = m_data->contact + i;

    int gid1 = con->geom1;
    int gid2 = con->geom2;
    int bid1 = m_model->geom_bodyid[gid1];
    int bid2 = m_model->geom_bodyid[gid2];

    ConstMJRigidBodyPtr body1
        = get_body(std::string(mj_id2name(m_model, mjOBJ_BODY, bid1)));
    ConstMJRigidBodyPtr body2
        = get_body(std::string(mj_id2name(m_model, mjOBJ_BODY, bid2)));

    kdl::ContactModel contact_model;
    if (con->dim == 1) {
      contact_model = kdl::ContactModel::FRICTIONLESS_POINT_CONTACT;
    }
    else if (con->dim == 3) {
      contact_model = kdl::ContactModel::POINT_CONTACT_WITH_FRICTION;
    }
    else if (con->dim == 4) {
      contact_model = kdl::ContactModel::SOFT_FINGER_CONTACT;
    }
    else {
      CROBOT_ASSERT_ERROR(
          false, "Contact model with dim = %d not supported.", con->dim);
    }

    math::Matrix3d xmat;
    xmat.block<3, 1>(0, 0)  = Eigen::Map<math::Vector3d>(con->frame + 3, 3);
    xmat.block<3, 1>(0, 1)  = Eigen::Map<math::Vector3d>(con->frame + 6, 3);
    xmat.block<3, 1>(0, 2)  = Eigen::Map<math::Vector3d>(con->frame + 0, 3);
    math::Matrix4d xform    = xmat.rotm2tform();
    xform.block<3, 1>(0, 3) = Eigen::Map<math::Vector3d>(con->pos, 3);

    math::Matrix4d xform1 = body1->get_xform();
    math::Matrix4d xform2 = body2->get_xform();

    math::Matrix4d tform1 = xform1.tform_inv() * xform;
    math::Matrix4d tform2 = xform2.tform_inv() * xform;

    contacts.push_back(std::make_shared<kdl::BodyContact>(
        contact_model,
        body1,
        std::make_shared<kdl::BodyFrame>(body1, tform1),
        body2,
        std::make_shared<kdl::BodyFrame>(body2, tform2),
        1.0));
  }

  return contacts;
}

//==============================================================================
void MJRBTree::add_body_mj(
    MJRigidBodyPtr body, const std::string& parent_name) {
  CROBOT_ASSERT_ERROR(body != nullptr);
  CROBOT_ASSERT_ERROR(
      body->m_joint != nullptr,
      "Bodies on a rigid body tree must have a valid joint.");

  // invoke the parent class add_body function
  kdl::RigidBodyTree::add_body(body, parent_name);

  // Get the joint
  MJJointPtr joint = std::dynamic_pointer_cast<MJJoint>(body->m_joint);

  // skip fixed joint
  if (joint->m_type == kdl::JointType::FIXED) {
    return;
  }

  // update qpos_addr
  std::pair<int, int> body_qpos_addr = joint->m_qpos_addr;
  CROBOT_ASSERT_ERROR(
      m_qpos_addr.first == -1
          || m_qpos_addr.first + m_qpos_addr.second == body_qpos_addr.first,
      "Wrong body orders when processing the body \"%s\". \n"
      "m_qpos_addr = (%d, %d), body_qpos_addr = (%d, %d)",
      body->get_name().c_str(),
      m_qpos_addr.first,
      m_qpos_addr.second,
      body_qpos_addr.first,
      body_qpos_addr.second);
  m_qpos_addr.first
      = m_qpos_addr.first == -1 ? body_qpos_addr.first : m_qpos_addr.first;
  m_qpos_addr.second += body_qpos_addr.second;

  // update qvel_addr
  std::pair<int, int> body_qvel_addr = joint->m_qvel_addr;
  CROBOT_ASSERT_ERROR(
      m_qvel_addr.first == -1
          || m_qvel_addr.first + m_qvel_addr.second == body_qvel_addr.first,
      "Wrong body orders when processing the body \"%s\". \n"
      "m_qvel_addr = (%d, %d), body_qvel_addr = (%d, %d)",
      body->get_name().c_str(),
      m_qvel_addr.first,
      m_qvel_addr.second,
      body_qvel_addr.first,
      body_qvel_addr.second);
  m_qvel_addr.first
      = m_qvel_addr.first == -1 ? body_qvel_addr.first : m_qvel_addr.first;
  m_qvel_addr.second += body_qvel_addr.second;
}

//==============================================================================
void MJRBTree::add_body_mj(int pid, int bid) {
  std::string pname
      = pid == 0 ? "world" : std::string(mj_id2name(m_model, mjOBJ_BODY, pid));
  int njnt = m_model->body_jntnum[bid];

  // update body_addr
  m_body_addr.insert(bid);

  // Case 1: No joint
  if (njnt == 0) {
    MJJointPtr joint    = MJJoint::MakeSharedFixed(m_model, bid);
    MJRigidBodyPtr body = MJRigidBody::MakeShared(m_model, bid, joint);
    add_body(body, pname);
    return;
  }

  // Case 2: One or more joints
  for (int j = 0; j < njnt; j++) {
    int jid = m_model->body_jntadr[bid] + j;

    // update joint_addr
    m_joint_addr.insert(jid);

    // Free Joint
    if (m_model->jnt_type[jid] == mjJNT_FREE) {
      CROBOT_ASSERT_ERROR(
          njnt == 1, "FREE joint should be the only joint in a body.");
      std::string jname = std::string(mj_id2name(m_model, mjOBJ_JOINT, jid));
      int qpos_addr     = m_model->jnt_qposadr[jid];
      int qvel_addr     = m_model->jnt_dofadr[jid];

      // Prismatic joint along the x-axis
      MJJointPtr joint_x = MJJoint::MakeSharedDummy(
          m_model,
          jname + "_x",
          kdl::JointType::PRISMATIC,
          qpos_addr,
          qvel_addr);
      joint_x->set_axis(math::Vector3d::UnitX());
      MJRigidBodyPtr body_x = MJRigidBody::MakeSharedDummy(
          m_model, jname + "_dummy_body_x", joint_x);
      add_body_mj(body_x, pname);
      pname = body_x->get_name();

      // Prismatic joint along the y-axis
      MJJointPtr joint_y = MJJoint::MakeSharedDummy(
          m_model,
          jname + "_y",
          kdl::JointType::PRISMATIC,
          qpos_addr + 1,
          qvel_addr + 1);
      joint_y->set_axis(math::Vector3d::UnitY());
      MJRigidBodyPtr body_y = MJRigidBody::MakeSharedDummy(
          m_model, jname + "_dummy_body_y", joint_y);
      add_body_mj(body_y, pname);
      pname = body_y->get_name();

      // Prismatic joint along the z-axis
      MJJointPtr joint_z = MJJoint::MakeSharedDummy(
          m_model,
          jname + "_z",
          kdl::JointType::PRISMATIC,
          qpos_addr + 2,
          qvel_addr + 2);
      joint_z->set_axis(math::Vector3d::UnitZ());
      MJRigidBodyPtr body_z = MJRigidBody::MakeSharedDummy(
          m_model, jname + "_dummy_body_z", joint_z);
      add_body_mj(body_z, pname);
      pname = body_z->get_name();

      // Ball joint
      MJJointPtr joint_ball = MJJoint::MakeSharedDummy(
          m_model,
          jname + "_ball",
          kdl::JointType::BALL,
          qpos_addr + 3,
          qvel_addr + 3);
      MJRigidBodyPtr body_ball
          = MJRigidBody::MakeShared(m_model, bid, joint_ball);
      add_body_mj(body_ball, pname);
      pname = body_ball->get_name();
    }
    // joints except the first and last one
    else if (j < njnt - 1) {
      // make a dummy body and add it to the rbtree
      MJJointPtr joint          = MJJoint::MakeShared(m_model, jid);
      MJRigidBodyPtr dummy_body = MJRigidBody::MakeSharedDummy(
          m_model, joint->get_name() + "_dummy_body", joint);
      add_body_mj(dummy_body, pname);
      pname = dummy_body->get_name();
    }
    // the last joint
    else {
      MJJointPtr joint    = MJJoint::MakeShared(m_model, jid);
      MJRigidBodyPtr body = MJRigidBody::MakeShared(m_model, bid, joint);
      add_body_mj(body, pname);
    }
  }
}

//==============================================================================
void MJRBTree::add_site_mj(int mjid_site) {
  int type            = m_model->site_type[mjid_site];
  int bodyid          = m_model->site_bodyid[mjid_site];
  math::Vector3d size = math::Vector3d::Map(m_model->site_size + mjid_site * 3);
  math::Vector3d bpos = math::Vector3d::Map(m_model->site_pos + mjid_site * 3);
  math::Quaternion quat = math::Quaternion(
      m_model->site_quat[mjid_site * 4 + 0],
      m_model->site_quat[mjid_site * 4 + 1],
      m_model->site_quat[mjid_site * 4 + 2],
      m_model->site_quat[mjid_site * 4 + 3]);

  std::string bodyname = std::string(mj_id2name(m_model, mjOBJ_BODY, bodyid));
  // skip if this site does not belong to this RigidBodyTree
  if (find_body_index_by_name(bodyname) == -1) {
    return;
  }

  MJRigidBodyPtr body
      = std::const_pointer_cast<MJRigidBody>(get_body(bodyname));
  if (body) {
    const char* site_name_cstr = mj_id2name(m_model, mjOBJ_SITE, mjid_site);
    std::string site_name;
    if (site_name_cstr == nullptr || std::strlen(site_name_cstr) == 0) {
      site_name = "site_" + std::to_string(mjid_site);
    }
    else {
      site_name = std::string(site_name_cstr);
    }
    body->add_site(
        site_name, static_cast<kdl::Site::Type>(type), bpos, quat, size);
  }
}

//==============================================================================
kdl::ConstTendonPtr MJRBTree::add_joint_tendon_mj(
    int mjid_tendon,
    const std::vector<int>& joint_ids,
    const std::vector<double>& coeffs) {
  m_tendon_addr.insert(mjid_tendon);

  std::string tendon_name
      = std::string(mj_id2name(m_model, mjOBJ_TENDON, mjid_tendon));
  std::vector<std::string> joint_names;
  joint_names.reserve(joint_ids.size());
  for (int jid : joint_ids) {
    joint_names.push_back(std::string(mj_id2name(m_model, mjOBJ_JOINT, jid)));
  }
  return kdl::RigidBodyTree::add_joint_tendon(tendon_name, joint_names, coeffs);
}

//==============================================================================
kdl::ConstActuatorPtr MJRBTree::add_joint_actuator_mj(
    int mjid_actuator, int mjid_joint) {
  CROBOT_ASSERT_ERROR(
      m_ctrl_addr.first == -1 ||                              // first actuator
      m_ctrl_addr.first + m_ctrl_addr.second == mjid_actuator // otherwiser
  );
  m_ctrl_addr.first
      = m_ctrl_addr.first == -1 ? mjid_actuator : m_ctrl_addr.first;
  m_ctrl_addr.second += 1;

  std::string actuator_name
      = std::string(mj_id2name(m_model, mjOBJ_ACTUATOR, mjid_actuator));
  std::string joint_name
      = std::string(mj_id2name(m_model, mjOBJ_JOINT, mjid_joint));
  return kdl::RigidBodyTree::add_joint_actuator(actuator_name, joint_name);
}

//==============================================================================
kdl::ConstActuatorPtr MJRBTree::add_tendon_actuator_mj(
    int mjid_actuator, int mjid_tendon) {
  CROBOT_ASSERT_ERROR(
      m_ctrl_addr.first == -1 ||                              // first actuator
      m_ctrl_addr.first + m_ctrl_addr.second == mjid_actuator // otherwiser
  );
  m_ctrl_addr.first
      = m_ctrl_addr.first == -1 ? mjid_actuator : m_ctrl_addr.first;
  m_ctrl_addr.second += 1;

  std::string actuator_name
      = std::string(mj_id2name(m_model, mjOBJ_ACTUATOR, mjid_actuator));
  std::string tendon_name
      = std::string(mj_id2name(m_model, mjOBJ_TENDON, mjid_tendon));
  return kdl::RigidBodyTree::add_tendon_actuator(actuator_name, tendon_name);
}

//==============================================================================
int MJRBTree::body_name2id(const std::string& bodyname) const {
  int mjbid = mj_name2id(m_model, mjOBJ_BODY, bodyname.c_str());
  CROBOT_ASSERT_ERROR(mjbid >= 0, "Unknown body name = %s.", bodyname.c_str());
  return mjbid;
}

//==============================================================================
math::Matrix4d MJRBTree::get_transform_mj(int mjbid) const {
  math::Matrix3d xmat  = Eigen::Map<math::Matrix3d>(m_data->xmat + mjbid * 9);
  math::Vector3d xpos  = Eigen::Map<math::Vector3d>(m_data->xpos + mjbid * 3);
  math::Matrix4d tform = math::Matrix4d::Identity();
  tform.block<3, 3>(0, 0) = xmat;
  tform.block<3, 1>(0, 3) = xpos;
  return tform;
}

//==============================================================================
math::Matrix4d MJRBTree::get_transform_mj(int mjbid1, int mjbid2) const {
  math::Matrix3d xmat1  = Eigen::Map<math::Matrix3d>(m_data->xmat + mjbid1 * 9);
  math::Vector3d xpos1  = Eigen::Map<math::Vector3d>(m_data->xpos + mjbid1 * 3);
  math::Matrix4d tform1 = math::Matrix4d::Identity();
  tform1.block<3, 3>(0, 0) = xmat1;
  tform1.block<3, 1>(0, 3) = xpos1;
  math::Matrix3d xmat2  = Eigen::Map<math::Matrix3d>(m_data->xmat + mjbid2 * 9);
  math::Vector3d xpos2  = Eigen::Map<math::Vector3d>(m_data->xpos + mjbid2 * 3);
  math::Matrix4d tform2 = math::Matrix4d::Identity();
  tform2.block<3, 3>(0, 0) = xmat2;
  tform2.block<3, 1>(0, 3) = xpos2;
  return tform2.tform_inv() * tform1;
}

//==============================================================================
math::LinearJacobian MJRBTree::get_body_jacp_mj(int bid) const {
  math::Matrix3X jacp(3, m_model->nv);
  mj_jacBody(m_model, m_data, jacp.data(), nullptr, bid);
  return jacp.middleCols(m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
math::LinearJacobian MJRBTree::get_body_jacp_at_xpos_mj(
    int bid, const math::Vector3d& xpos) const {
  math::Matrix3X jacp(3, m_model->nv);
  mj_jac(m_model, m_data, jacp.data(), nullptr, xpos.data(), bid);
  return jacp.middleCols(m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
math::AngularJacobian MJRBTree::get_body_jacr_mj(int bid) const {
  math::Matrix3X jacr(3, m_model->nv);
  mj_jacBody(m_model, m_data, nullptr, jacr.data(), bid);
  return jacr.middleCols(m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
math::Jacobian MJRBTree::get_body_jacb_mj(int bid) const {
  math::Matrix3X jacp(3, m_model->nv);
  math::Matrix3X jacr(3, m_model->nv);
  mj_jacBody(m_model, m_data, jacp.data(), jacr.data(), bid);
  math::Jacobian jacb(6, m_model->nv);
  jacb.topRows(3)    = jacr;
  jacb.bottomRows(3) = jacp;
  return jacb.middleCols(m_qvel_addr.first, m_qvel_addr.second);
}

//==============================================================================
math::Jacobian MJRBTree::get_body_jacb_at_xpos(
    int bid, const math::Vector3d& xpos) const {
  math::Matrix3X jacp(3, m_model->nv);
  math::Matrix3X jacr(3, m_model->nv);
  mj_jac(m_model, m_data, jacp.data(), jacr.data(), xpos.data(), bid);
  math::Jacobian jacb(6, m_model->nv);
  jacb.topRows(3)    = jacr;
  jacb.bottomRows(3) = jacp;
  return jacb;
}

//==============================================================================
math::Matrix3X MJRBTree::get_body_jacp_dot(int bid) const {
  CROBOT_ASSERT_ERROR(
      m_qpos_addr.second == m_qvel_addr.second,
      "qpos dimension does not match qvel dimension, so that simple numerical "
      "differentiation is not suitable for Jacobian time derivative "
      "computation.");

  double dt                  = 0.002;
  math::VectorXd qpos_center = math::VectorXd::Map(
      m_data->qpos + m_qpos_addr.first, m_qpos_addr.second);
  math::VectorXd qvel = math::VectorXd::Map(
      m_data->qvel + m_qvel_addr.first, m_qvel_addr.second);

  // Calculate the Jacobian at (t - dt)
  math::VectorXd qpos_prev = qpos_center - qvel * dt;
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_prev;
  math::LinearJacobian jacp_prev = get_body_jacp_mj(bid);

  // Calculate the Jacobian at (t + dt)
  math::VectorXd qpos_next = qpos_center + qvel * dt;
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_next;
  math::LinearJacobian jacp_next = get_body_jacp_mj(bid);

  // recover the original qpos m_data
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_center;

  return (jacp_next - jacp_prev) / (2. * dt);
}

//==============================================================================
math::Matrix3X MJRBTree::get_body_jacr_dot(int bid) const {
  CROBOT_ASSERT_ERROR(
      m_qpos_addr.second == m_qvel_addr.second,
      "qpos dimension does not match qvel dimension, so that simple numerical "
      "differentiation is not suitable for Jacobian time derivative "
      "computation.");

  double dt                  = 0.002;
  math::VectorXd qpos_center = math::VectorXd::Map(
      m_data->qpos + m_qpos_addr.first, m_qpos_addr.second);
  math::VectorXd qvel = math::VectorXd::Map(
      m_data->qvel + m_qvel_addr.first, m_qvel_addr.second);

  // Calculate the Jacobian at (t - dt)
  math::VectorXd qpos_prev = qpos_center - qvel * dt;
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_prev;
  math::LinearJacobian jacr_prev = get_body_jacr_mj(bid);

  // Calculate the Jacobian at (t + dt)
  math::VectorXd qpos_next = qpos_center + qvel * dt;
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_next;
  math::LinearJacobian jacr_next = get_body_jacr_mj(bid);

  // recover the original qpos m_data
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_center;

  return (jacr_next - jacr_prev) / (2. * dt);
}

//==============================================================================
math::Matrix6X MJRBTree::get_body_jacb_dot(int bid) const {
  CROBOT_ASSERT_ERROR(
      m_qpos_addr.second == m_qvel_addr.second,
      "qpos dimension does not match qvel dimension, so that simple numerical "
      "differentiation is not suitable for Jacobian time derivative "
      "computation.");

  double dt                  = 0.002;
  math::VectorXd qpos_center = math::VectorXd::Map(
      m_data->qpos + m_qpos_addr.first, m_qpos_addr.second);
  math::VectorXd qvel = math::VectorXd::Map(
      m_data->qvel + m_qvel_addr.first, m_qvel_addr.second);

  // Calculate the Jacobian at (t - dt)
  math::VectorXd qpos_prev = qpos_center - qvel * dt;
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_prev;
  math::Matrix6X jacb_prev = get_body_jacb_mj(bid);

  // Calculate the Jacobian at (t + dt)
  math::VectorXd qpos_next = qpos_center + qvel * dt;
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_next;
  math::Matrix6X jacb_next = get_body_jacb_mj(bid);

  // recover the original qpos m_data
  math::VectorXd::Map(m_data->qpos + m_qpos_addr.first, m_qpos_addr.second)
      = qpos_center;

  return (jacb_next - jacb_prev) / (2. * dt);
}
//==============================================================================
void MJRBTree::set_external_wrench(
    int bid, const math::Vector6d& wrench) const {
  MJRigidBodyPtr body = std::dynamic_pointer_cast<MJRigidBody>(m_bodies[bid]);
  int mjid            = body->m_mjid;
  CROBOT_ASSERT_ERROR(
      mjid != -1, "Cannot apply an external wrench to a dummy body.");

  // special case: when the given wrench is a zero vector, no need to transform
  if (wrench.isZero()) {
    math::Vector6d::Map(m_data->xfrc_applied + mjid * 6) = wrench;
    return;
  }

  // transform the spatial wrench in the torque-force representation
  math::Matrix4d Tsb = get_transform_mj(mjid);
  math::Vector6d Ws  = wrench;
  math::Vector6d Wb  = Tsb.tform2xform().transpose() * Ws;
  math::Vector3d ms  = Tsb.tform2rotm() * Wb.segment(0, 3);
  math::Vector3d fs  = Tsb.tform2rotm() * Wb.segment(3, 3);

  m_data->xfrc_applied[mjid * 6 + 0] = fs[0]; // Fx
  m_data->xfrc_applied[mjid * 6 + 1] = fs[1]; // Fy
  m_data->xfrc_applied[mjid * 6 + 2] = fs[2]; // Fz
  m_data->xfrc_applied[mjid * 6 + 3] = ms[0]; // Tx
  m_data->xfrc_applied[mjid * 6 + 4] = ms[1]; // Ty
  m_data->xfrc_applied[mjid * 6 + 5] = ms[2]; // Tz
}

//==============================================================================
void MJRBTree::set_xtorq_xfrc(
    int bid, const math::Vector3d& xtorq, const math::Vector3d& xfrc) const {
  MJRigidBodyPtr body = std::dynamic_pointer_cast<MJRigidBody>(m_bodies[bid]);
  int mjid            = body->m_mjid;
  CROBOT_ASSERT_ERROR(
      mjid != -1, "Cannot apply an external wrench to a dummy body.");

  m_data->xfrc_applied[mjid * 6 + 0] = xfrc[0];  // Fx
  m_data->xfrc_applied[mjid * 6 + 1] = xfrc[1];  // Fy
  m_data->xfrc_applied[mjid * 6 + 2] = xfrc[2];  // Fz
  m_data->xfrc_applied[mjid * 6 + 3] = xtorq[0]; // Tx
  m_data->xfrc_applied[mjid * 6 + 4] = xtorq[1]; // Ty
  m_data->xfrc_applied[mjid * 6 + 5] = xtorq[2]; // Tz
}
