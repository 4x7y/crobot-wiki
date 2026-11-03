#include "crobot/control/rolling_action2.hpp"

#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace control;

//==============================================================================
RollingAction2::RollingAction2(
    kdl::ConstRigidBodyTreePtr rbtree,
    kdl::ConstRigidBodyPtr body,
    kdl::DynamicGrasp::ConstPtr grasp_target,
    const planning::PoseTrajectory::ConstPtr& body_traj,
    const math::Vector6d& xwrench_ext_desired,
    const std::vector<ContactTarget>& contact_targets)
  : Action("RollingAction2", body_traj->get_duration()),
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
std::pair<math::VectorXd, math::VectorXd> RollingAction2::get_command(
    double time,
    const math::VectorXd& qpos_actual,
    const math::VectorXd& qpos_cmd,
    const math::VectorXd& qvel_actual) {

  std::vector<math::Matrix4d> tforms_cmd
      = m_rbtree->forward_kinematics(qpos_cmd);

  // get desired pose of the screwdriver from the trajectory
  m_obj_ctrl_desired_xframe->set_xform(m_body_traj->get_tform(time));
  m_obj_ctrl_desired_xframe->set_xvel(m_body_traj->get_velocity(time));
  m_obj_ctrl_desired_xframe->set_xacc(m_body_traj->get_acceleration(time));

  math::Matrix4d xform_obj_actual    = math::Matrix4d::Identity();
  xform_obj_actual.block<3, 3>(0, 0) = math::Quaternion(
                                           qpos_actual[m_target_qpos_addr[3]],
                                           qpos_actual[m_target_qpos_addr[4]],
                                           qpos_actual[m_target_qpos_addr[5]],
                                           qpos_actual[m_target_qpos_addr[6]])
                                           .toRotationMatrix();
  xform_obj_actual.block<3, 1>(0, 3)
      = qpos_actual.segment<3>(m_target_qpos_addr[0]);

  math::Matrix4d xform_obj_cmd    = math::Matrix4d::Identity();
  xform_obj_cmd.block<3, 3>(0, 0) = math::Quaternion(
                                        qpos_cmd[m_target_qpos_addr[3]],
                                        qpos_cmd[m_target_qpos_addr[4]],
                                        qpos_cmd[m_target_qpos_addr[5]],
                                        qpos_cmd[m_target_qpos_addr[6]])
                                        .toRotationMatrix();
  xform_obj_cmd.block<3, 1>(0, 3) = qpos_cmd.segment<3>(m_target_qpos_addr[0]);

  math::Matrix4d xform_obj_desired
      = m_obj_ctrl->resolve(xform_obj_actual, xform_obj_cmd);

  // info about controlled body
  // math::Matrix3d xmat_o = m_body->get_xmat(tforms);

  // update contacts
  auto contacts            = m_grasp->get_dynamic_contacts();
  bool is_contact_boundary = false;
  for (kdl::BodyRollingContact::Ptr contact : contacts) {
    // auto ctc_dir = contact->get_rolling_moving_dir(tforms,
    // qvel_actual_updated);
    auto ctc_dir = contact->get_rolling_moving_dir(tforms_cmd, qvel_actual);

    contact->update_contact_points_in_dir(ctc_dir.first, ctc_dir.second, m_dt);
    is_contact_boundary |= contact->is_boundary_contact();

    if (is_contact_boundary) {
      CROBOT_LOG_WARN(
          "Contact boundary of {} detected. Rolling action is aborted.",
          contact->get_body2()->get_name());
      m_is_active = false;
      return std::make_pair(qpos_actual, qpos_cmd);
    }
  }

  int num_wrench_biasis         = m_grasp->get_num_wrench_basis();
  math::Matrix6X mat_grasp      = m_grasp->get_grasp_matrix();
  math::MatrixXd mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);
  math::MatrixXd mat_null_space_mapping
      = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
        - mat_grasp_pinv * mat_grasp;

  // solve for the desired bacc and bwrench on the body
  // math::Vector6d bwrench_resolved = math::Vector6d::Zero();
  // bwrench_resolved.head<3>()
  //     = xmat_o.transpose() * m_xwrench_ext_desired.tail<3>();
  // bwrench_resolved.tail<3>()
  //     = xmat_o.transpose() * m_xwrench_ext_desired.head<3>();

  // math::VectorXd cwrench_resolved = mat_grasp_pinv * bwrench_resolved;

  math::VectorXd cfrc_internal = mat_null_space_mapping * m_cfrc_internal_stack;

  // angular velocity of the object

  for (size_t i = 0; i < m_ctrl_targets.size(); ++i) {
    control::AdmittanceControlTarget::Ptr ctrl_target;
    kdl::WorldFramePtr xframe_desired;
    std::tie(ctrl_target, xframe_desired) = m_ctrl_targets[i];
    std::string body_name = m_ctrl_targets[i].first->get_name(2);
    kdl::BodyRollingContact::Ptr contact
        = m_grasp->get_dynamic_contact(body_name);

    xframe_desired->set_xform(
        xform_obj_desired * contact->get_frame1()->get_tform());

    std::size_t contact_id_grasp = m_name_to_index[body_name];
    // math::Vector3d cfrc   = cwrench_resolved.segment(contact_id_grasp * 3,
    // 3);
    math::Vector3d cfrc_i = cfrc_internal.segment(contact_id_grasp * 3, 3);
    // ctrl_target->set_desired_force(cfrc + cfrc_i);
    ctrl_target->set_desired_force(cfrc_i);
  }

  // for visualization
  math::VectorXd qpos_body(7);
  qpos_body.head<3>() = xform_obj_desired.tform2trvec();
  qpos_body.tail<4>() = xform_obj_desired.tform2quat().as_vector();

  math::VectorXd qpos_actual_ret, qpos_cmd_ret;

  if (!m_controller->compute(qpos_actual, qpos_cmd, qvel_actual)) {
    m_is_active = false;
    return std::make_pair(qpos_actual, qpos_cmd);
  }

  std::tie(qpos_actual_ret, qpos_cmd_ret) = m_controller->get_result();
  qpos_actual_ret(m_target_qpos_addr)     = qpos_body;
  qpos_cmd_ret(m_target_qpos_addr)        = qpos_body;
  return std::make_pair(qpos_actual_ret, qpos_cmd_ret);
}

//==============================================================================
void RollingAction2::initialize(
    const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
  (void)qpos_cmd;

  CROBOT_LOG_INFO("Initializing SmoothAction: {}", get_name());
  m_is_inited = true;
  m_is_active = true;
  m_type      = ROLLING;

  std::vector<math::Matrix4d> tforms
      = m_rbtree->forward_kinematics(qpos_actual);

  std::vector<kdl::BodyContact::Ptr> contacts = m_grasp->get_contacts();
  m_cfrc_internal_stack = math::VectorXd::Zero(3 * contacts.size());

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

  m_obj_ctrl_desired_xframe = std::make_shared<kdl::WorldFrame>();
  m_obj_ctrl                = std::make_shared<control::ObjXposCtrlTarget>(
      m_obj_ctrl_desired_xframe,
      m_body,
      math::Matrix3d::Identity(),
      math::Vector3d::Zero(),
      "/ws/resources/control_params/ekf_ctrl_param.yaml");

  CROBOT_LOG_INFO("Completed initializing PivotAction: {}", get_name());
}

//==============================================================================
void RollingAction2::fix_joint(const std::string& joint_name) {
  m_controller->fix_joint(joint_name);
}

//==============================================================================
void RollingAction2::fix_joint(const std::string& joint_name, double value) {
  m_controller->fix_joint(joint_name, value);
}

//==============================================================================
void RollingAction2::fix_body(const std::string& body_name) {
  m_controller->fix_body(body_name);
}

//==============================================================================
kdl::Grasp::ConstPtr RollingAction2::get_grasp() const {
  return m_grasp;
}
//==============================================================================
void RollingAction2::set_null_space_obj(ik::NullSpaceObj::Ptr obj) {
  m_controller->set_null_space_objective(obj);
}
