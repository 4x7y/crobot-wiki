#include "crobot/control/rolling_action.hpp"

#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
RollingAction::RollingAction(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::ConstRigidBodyPtr body,
    kdl::DynamicGrasp::ConstPtr grasp_target,
    const planning::PoseTrajectory::ConstPtr& body_traj,
    const math::Vector6d& xwrench_ext_desired,
    const std::vector<ContactTarget>& contact_targets)
  : Action("RollingAction", body_traj->get_duration()),
    m_rbtree(rbtree),
    m_body(body),
    m_grasp(grasp_target),
    m_body_traj(body_traj),
    m_xwrench_ext_desired(xwrench_ext_desired),
    m_contact_targets(contact_targets) {

  m_controller       = std::make_shared<control::AdmittanceController>(rbtree);
  m_target_qpos_addr = m_body->get_support_qpos_addr();
  m_target_qvel_addr = m_body->get_support_qvel_addr();
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> RollingAction::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);

  // get desired pose of the screwdriver from the trajectory
  math::Matrix4d xform_sd_desired = m_body_traj->get_tform(time);
  math::VectorXd xvel_sd_desired  = m_body_traj->get_velocity(time);
  math::VectorXd xacc_sd_desired  = m_body_traj->get_acceleration(time);

  // info about controlled body
  math::Matrix3d xmat_o = m_body->get_xmat(tforms);
  // math::Vector3d xvelr_o = xvel_sd_desired.head<3>();
  // math::Vector3d xvelp_o = xvel_sd_desired.tail<3>();
  // math::Vector6d xvel_pr;
  // xvel_pr <<  xvelp_o, xvelr_o;
  // math::VectorXd qvel_actual_updated      = qvel_actual;
  // qvel_actual_updated(m_target_qvel_addr) = xvel_pr;

  // update contacts
  auto contacts            = m_grasp->get_dynamic_contacts();
  bool is_contact_boundary = false;
  for (kdl::BodyRollingContact::Ptr contact : contacts) {
    // auto ctc_dir = contact->get_rolling_moving_dir(tforms,
    // qvel_actual_updated);
    auto ctc_dir = contact->get_rolling_moving_dir(tforms, qvel_actual);

    contact->update_contact_points_in_dir(ctc_dir.first, ctc_dir.second, m_dt);
    is_contact_boundary |= contact->is_boundary_contact();
  }

  if (is_contact_boundary) {
    CROBOT_LOG_WARN(
        "Contact boundary of {} detected. Rolling action is aborted.",
        contacts[0]->get_body2()->get_name());
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  int num_wrench_biasis         = m_grasp->get_num_wrench_basis();
  math::Matrix6X mat_grasp      = m_grasp->get_grasp_matrix();
  math::MatrixXd mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);
  math::MatrixXd mat_null_space_mapping
      = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
        - mat_grasp_pinv * mat_grasp;

  // solve for the desired bacc and bwrench on the body
  math::Vector6d bwrench_resolved = math::Vector6d::Zero();
  bwrench_resolved.head<3>()
      = xmat_o.transpose() * m_xwrench_ext_desired.tail<3>();
  bwrench_resolved.tail<3>()
      = xmat_o.transpose() * m_xwrench_ext_desired.head<3>();

  math::VectorXd cwrench_resolved = mat_grasp_pinv * bwrench_resolved;

  math::VectorXd cfrc_internal = mat_null_space_mapping * m_cfrc_internal_stack;

  // angular velocity of the object

  for (size_t i = 0; i < m_ctrl_targets.size(); ++i) {
    control::AdmittanceControlTarget::Ptr ctrl_target;
    kdl::WorldFramePtr xframe_desired;
    std::tie(ctrl_target, xframe_desired) = m_ctrl_targets[i];
    std::string body_name = m_ctrl_targets[i].first->get_name(2);
    kdl::BodyRollingContact::Ptr contact
        = m_grasp->get_dynamic_contact(body_name);

    math::Matrix4d xform_obj = m_body->get_xform(tforms);
    xframe_desired->set_xform(xform_obj * contact->get_frame1()->get_tform());

    std::size_t contact_id_grasp = m_name_to_index[body_name];
    math::Vector3d cfrc   = cwrench_resolved.segment(contact_id_grasp * 3, 3);
    math::Vector3d cfrc_i = cfrc_internal.segment(contact_id_grasp * 3, 3);
    ctrl_target->set_desired_force(cfrc + cfrc_i);

    // std::cout << "contact at " << body_name << ":" << (cfrc + cfrc_i).norm()
    //           << std::endl;

    // update contacts
    // auto ctc_dir = contact->get_rolling_moving_dir(tforms,
    // qvel_actual_updated);

    // contact->update_contact_points_in_dir(ctc_dir.first, ctc_dir.second,
    // m_dt);
  }

  // for visualization
  math::VectorXd qpos_body(7);
  // qpos_body.head<3>() = m_body->get_xpos(tforms);
  // qpos_body.tail<4>() = m_body->get_xquat(tforms).as_vector();
  qpos_body.head<3>() = xform_sd_desired.tform2trvec();
  qpos_body.tail<4>() = xform_sd_desired.tform2quat().as_vector();

  math::VectorXd qpos_actual_ret, qpos_cmd_ret;

  if (!m_controller->compute(qpos_actual, qpos_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  std::tie(qpos_actual_ret, qpos_cmd_ret) = m_controller->get_result();

  qpos_actual_ret(m_target_qpos_addr) = qpos_body;
  qpos_cmd_ret(m_target_qpos_addr)    = qpos_body;
  return std::make_pair(qpos_actual_ret, qpos_cmd_ret);
}

//==============================================================================
void RollingAction::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  (void)qpos_cmd;

  CROBOT_LOG_INFO("Initializing SmoothAction: {}", get_name());
  m_is_inited = true;
  m_is_active = true;
  m_type      = ROLLING;

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);

  // m_body_target_desired_xframe = std::make_shared<kdl::WorldFrame>();
  // m_body_target                = BodyTarget::MakeShared(
  //     m_body_target_desired_xframe,
  //     m_body,
  //     m_rolling_target.brotm,
  //     m_rolling_target.bpos,
  //     m_rolling_target.param_file_name);

  std::vector<kdl::BodyContact::Ptr> contacts = m_grasp->get_contacts();
  m_cfrc_internal_stack = math::VectorXd::Zero(3 * contacts.size());

  // CROBOT_ASSERT_ERROR(
  //     contacts.size() == m_contact_targets.size(),
  //     "ALL contacts must be under control!");

  for (std::size_t k = 0; k < contacts.size(); k++) {
    kdl::BodyContact::Ptr contact = contacts[k];
    for (std::size_t i = 0; i < m_contact_targets.size(); i++) {
      if (contact->get_body2()->get_name() == m_contact_targets[i].body_name) {
        m_cfrc_internal_stack.segment(k * 3, 3)
            = m_contact_targets[i].bfrc_desired;
        m_name_to_index[contact->get_body2()->get_name()] = k;
        break;
      }
    }
  }

  for (std::size_t i = 0; i < m_contact_targets.size(); i++) {
    const std::string body_name = m_contact_targets[i].body_name;
    kdl::BodyRollingContact::Ptr contact
        = m_grasp->get_dynamic_contact(body_name);

    kdl::WorldFrame::Ptr xframe_desired = std::make_shared<kdl::WorldFrame>();
    control::AdmittanceControlTarget::Ptr ctrl_target
        = m_controller->add_rolling_target(
            xframe_desired,
            contact->get_body2()->get_name(),
            math::EulerAngleSeq::ZYX,
            m_contact_targets[i]
                .dofs_controlled, // rolling only control translational DOFs
            contact);
    ctrl_target->set_desired_force(m_contact_targets[i].bfrc_desired);

    math::Matrix4d xform_ctc = contact->get_body2()->get_xform(tforms)
                               * contact->get_frame2()->get_tform();
    xframe_desired->set_xform(xform_ctc);
    m_ctrl_targets.push_back(std::make_pair(ctrl_target, xframe_desired));
  }
  CROBOT_LOG_INFO("Completed initializing PivotAction: {}", get_name());
}

//==============================================================================
void RollingAction::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void RollingAction::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void RollingAction::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}

//==============================================================================
kdl::Grasp::ConstPtr RollingAction::get_grasp() const {
  return m_grasp;
}
//==============================================================================
void RollingAction::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
