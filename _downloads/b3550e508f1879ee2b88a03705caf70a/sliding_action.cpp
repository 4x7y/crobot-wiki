// #include "crobot/control/sliding_action.hpp"

// #include "crobot/common/logging.hpp"
// #include "crobot/planning/linear_trajectory.hpp"
// #include "crobot/planning/trapvel_time_scaling.hpp"

// using namespace crobot;
// using namespace control;

// //==============================================================================
// SlidingAction::SlidingAction(
//     kdl::ConstRigidBodyTreePtr rbtree,
//     kdl::DynamicGrasp::ConstPtr grasp_prev,
//     kdl::DynamicGrasp::ConstPtr grasp_next,
//     const std::vector<CtrlTarget>& moving_targets,
//     const std::vector<CtrlTarget>& force_targets,
//     const math::Vector6d& xwrench_ext_desired,
//     double duration)
//   : Action("SlidingAction", duration),
//     m_rbtree(rbtree),
//     m_grasp(grasp_prev),
//     m_grasp_target(grasp_next),
//     m_moving_targets(moving_targets),
//     m_force_targets(force_targets),
//     m_xwrench_ext_desired(xwrench_ext_desired) {
//   m_controller = std::make_shared<control::AdmittanceController>(rbtree);
//   m_body
//       =
//       m_rbtree->get_body(m_grasp->get_contacts()[0]->get_body1()->get_name());

//   CROBOT_ASSERT_ERROR(
//       m_body != nullptr,
//       "Sliding action initialization: target body not found");
// }

// //==============================================================================
// std::pair<math::VectorXd, math::VectorXd> SlidingAction::get_command(
//     double time,
//     const math::VectorXd& qpos_actual,
//     const math::VectorXd& qpos_cmd,
//     const math::VectorXd& qvel_actual) {

//   std::vector<math::Matrix4d> tforms
//       = m_rbtree->forward_kinematics(qpos_actual);

//   // all contacts
//   std::vector<kdl::BodyRollingContact::Ptr> contacts
//       = m_grasp->get_dynamic_contacts();

//   // for (auto& contact : contacts) {
//   //   // update contacts
//   //   auto ctc_dir = contact->get_moving_dir(tforms, qvel_actual);
//   //   contact->update_contact_points_in_dir(ctc_dir.first, ctc_dir.second,
//   //   m_dt);
//   // }

//   int num_wrench_biasis         = m_grasp->get_num_wrench_basis();
//   math::Matrix6X mat_grasp      = m_grasp->get_grasp_matrix();
//   math::MatrixXd mat_grasp_pinv = math::calculate_jacb_inv(mat_grasp);
//   math::MatrixXd mat_null_space_mapping
//       = math::MatrixXd::Identity(num_wrench_biasis, num_wrench_biasis)
//         - mat_grasp_pinv * mat_grasp;

//   // get desired pose of the screwdriver from the trajectory
//   // math::Matrix4d xform_object = m_body->get_xform(tforms);

//   // solve for the desired bacc and bwrench on the body
//   math::Vector6d bwrench_resolved = m_xwrench_ext_desired;
//   bwrench_resolved.head<3>().swap(bwrench_resolved.tail<3>());

//   math::VectorXd cwrench_resolved = mat_grasp_pinv * bwrench_resolved;
//   math::VectorXd cfrc_internal = mat_null_space_mapping *
//   m_cfrc_internal_stack;

//   // update moving targets contact, so the desired xframe will be updated
//   next for (size_t i = 0; i < m_moving_targets.size(); ++i) {
//     std::string body_name                = m_moving_targets[i].body_name;
//     std::size_t contact_id_grasp         = m_name_to_index[body_name];
//     kdl::BodyRollingContact::Ptr contact = contacts[contact_id_grasp];

//     // math::Vector3d bpos1_target = m_bpos_targets[i].first;
//     // math::Vector3d bpos1_now    = contact->get_frame1()->get_bpos();
//     // math::Vector3d bpos2_target = m_bpos_targets[i].second;
//     // math::Vector3d bpos2_now    = contact->get_frame2()->get_bpos();

//     math::Vector3d bvel1 = m_bpos_targets[i].first->get_velocity(time);
//     math::Vector3d bvel2 = m_bpos_targets[i].second->get_velocity(time);

//     math::Vector3d cvel1 = contact->get_surface1()
//                                ->get_frame(contact->get_contact_points().first)
//                                .tform2rotm()
//                                .transpose()
//                            * bvel1;
//     math::Vector3d cvel2 = contact->get_surface2()
//                                ->get_frame(contact->get_contact_points().second)
//                                .tform2rotm()
//                                .transpose()
//                            * bvel2;

//     math::Vector3d cvel1_t(cvel1[0], cvel1[1], 0.0);
//     math::Vector3d cvel2_t(cvel2[0], cvel2[1], 0.0);

//     if (cvel1_t.norm() > 1e-5)
//       cvel1
//           = cvel1_t * (std::pow(cvel1.norm(), 2) / std::pow(cvel1_t.norm(),
//           2));
//     else
//       cvel1 = math::Vector3d::Zero();

//     if (cvel2_t.norm() > 1e-5)
//       cvel2
//           = cvel2_t * (std::pow(cvel2.norm(), 2) / std::pow(cvel2_t.norm(),
//           2));
//     else
//       cvel2 = math::Vector3d::Zero();

//     std::cout << body_name << " cvel1: " << cvel1.transpose() << std::endl;
//     std::cout << body_name << " cvel2: " << cvel2.transpose() << std::endl;

//     // frame at the object has flipped normal, but jacb is not flipped
//     math::Vector2d move_dir1
//         = math::calculate_jacb_inv(contact->get_jacb1(), 0.0001) * cvel1;
//     math::Vector2d move_dir2
//         = math::calculate_jacb_inv(contact->get_jacb2(), 0.0001) * cvel2;

//     std::cout << body_name << " move_dir1: " << move_dir1.transpose()
//               << std::endl;
//     std::cout << body_name << " move_dir2: " << move_dir2.transpose()
//               << std::endl;

//     // math::Vector2d move_dir1
//     //     = contact->get_jacb1().colPivHouseholderQr().solve(bvel1);
//     // math::Vector2d move_dir2
//     //     = contact->get_jacb2().colPivHouseholderQr().solve(bvel2);

//     std::cout << "m_dt = " << m_dt << std::endl;
//     contact->update_contact_points_in_dir(move_dir1, move_dir2, m_dt);
//   }

//   // update desired forces
//   for (size_t i = 0; i < m_ctrl_targets.size(); ++i) {
//     control::AdmittanceControlTarget::Ptr ctrl_target;
//     kdl::WorldFramePtr xframe_desired;
//     std::tie(ctrl_target, xframe_desired) = m_ctrl_targets[i];
//     std::string body_name        = m_ctrl_targets[i].first->get_name(2);
//     std::size_t contact_id_grasp = m_name_to_index[body_name];

//     kdl::BodyRollingContact::Ptr contact
//         = m_grasp->get_dynamic_contact(body_name);
//     math::Matrix3d xmat_ctc_obj   = contact->get_xmat1(tforms);
//     math::Matrix3d xmat_ctc_manip = contact->get_xmat2(tforms);
//     math::Vector3d bpos_ctc_obj   = contact->get_frame1()->get_bpos();
//     math::Matrix3d bmat_ctc_obj   = contact->get_frame1()->get_rotm();

//     math::Vector3d xpos_now    = m_body->get_xpos(tforms, bpos_ctc_obj);
//     math::Matrix3d xmat_now    = m_body->get_xmat(tforms, bmat_ctc_obj);
//     math::Matrix4d xform_now   = xmat_now.rotm2tform();
//     xform_now.col(3).head<3>() = xpos_now;
//     xframe_desired->set_xform(xform_now);

//     math::Vector3d cfrc = xmat_ctc_manip.transpose() * xmat_ctc_obj
//                           * cwrench_resolved.segment(contact_id_grasp * 3,
//                           3);

//     // TODO: add friction compensation

//     ctrl_target->set_desired_force(
//         cfrc + cfrc_internal.segment(contact_id_grasp * 3, 3));

//     // auto ctc_dir = contact->get_moving_dir(tforms, qvel_actual);
//     // contact->update_contact_points_in_dir(ctc_dir.first, ctc_dir.second,
//     // m_dt);
//   }

//   if (!m_controller->compute(qpos_actual, qpos_cmd, qvel_actual)) {
//     m_is_active = false;
//     return std::make_pair(qpos_actual, qpos_cmd);
//   }

//   return m_controller->get_result();
// }

// //==============================================================================
// void SlidingAction::initialize(
//     const math::VectorXd& qpos_actual, const math::VectorXd& qpos_cmd) {
//   (void)qpos_cmd;
//   m_is_inited = true;
//   m_is_active = true;
//   m_type      = SLIDING;

//   std::vector<math::Matrix4d> tforms
//       = m_rbtree->forward_kinematics(qpos_actual);

//   std::vector<kdl::BodyContact::Ptr> contacts = m_grasp->get_contacts();
//   m_cfrc_internal_stack = math::VectorXd::Zero(3 * contacts.size());

//   CROBOT_ASSERT_ERROR(
//       contacts.size() == m_moving_targets.size() + m_force_targets.size(),
//       "ALL contacts must be under control!");

//   for (std::size_t k = 0; k < contacts.size(); k++) {
//     kdl::BodyContact::Ptr contact = contacts[k];
//     for (std::size_t i = 0; i < m_moving_targets.size(); i++) {
//       if (contact->get_body2()->get_name() == m_moving_targets[i].body_name)
//       {
//         m_cfrc_internal_stack.segment(k * 3, 3)
//             = m_moving_targets[i].bfrc_desired;
//         m_name_to_index[contact->get_body2()->get_name()] = k;
//         break;
//       }
//     }

//     for (std::size_t i = 0; i < m_force_targets.size(); i++) {
//       if (contact->get_body2()->get_name() == m_force_targets[i].body_name) {
//         m_cfrc_internal_stack.segment(k * 3, 3)
//             = m_force_targets[i].bfrc_desired;
//         m_name_to_index[contact->get_body2()->get_name()] = k;
//         break;
//       }
//     }
//   }

//   m_bpos_targets.resize(m_moving_targets.size());

//   planning::TrapvelTimeScaling::ConstPtr time_scaling
//       = std::make_shared<planning::TrapvelTimeScaling>(get_duration());

//   for (std::size_t i = 0; i < m_moving_targets.size(); i++) {
//     set_control_targets(m_moving_targets[i], tforms);
//     kdl::BodyRollingContact::Ptr contact_target
//         = m_grasp_target->get_dynamic_contact(m_moving_targets[i].body_name);
//     kdl::BodyRollingContact::Ptr contact_now
//         = m_grasp->get_dynamic_contact(m_moving_targets[i].body_name);
//     math::Vector3d bpos1     = contact_target->get_frame1()->get_bpos();
//     math::Vector3d bpos2     = contact_target->get_frame2()->get_bpos();
//     math::Vector3d bpos1_now = contact_now->get_frame1()->get_bpos();
//     math::Vector3d bpos2_now = contact_now->get_frame2()->get_bpos();
//     m_bpos_targets[i]        = std::make_pair(
//         std::make_shared<planning::LinearTrajectory>(
//             bpos1_now, bpos1, time_scaling),
//         std::make_shared<planning::LinearTrajectory>(
//             bpos2_now, bpos2, time_scaling));
//   }

//   for (std::size_t i = 0; i < m_force_targets.size(); i++) {
//     set_control_targets(m_force_targets[i], tforms);
//   }
// }

// //==============================================================================
// void SlidingAction::fix_joint(const std::string& joint_name) {
//   m_controller->fix_joint(joint_name);
// }

// //==============================================================================
// void SlidingAction::fix_joint(const std::string& joint_name, double value) {
//   m_controller->fix_joint(joint_name, value);
// }

// //==============================================================================
// void SlidingAction::fix_body(const std::string& body_name) {
//   m_controller->fix_body(body_name);
// }

// //==============================================================================
// void SlidingAction::set_control_targets(
//     CtrlTarget& target, std::vector<math::Matrix4d>& tforms) {
//   const std::string body_name = target.body_name;
//   kdl::BodyRollingContact::Ptr contact
//       = m_grasp->get_dynamic_contact(body_name);

//   kdl::WorldFrame::Ptr xframe_desired = std::make_shared<kdl::WorldFrame>();
//   control::AdmittanceControlTarget::Ptr adm_ctrl_target
//       = m_controller->add_rolling_target(
//           xframe_desired,
//           contact->get_body2()->get_name(),
//           math::EulerAngleSeq::ZYX,
//           target.dofs_controlled,
//           contact);
//   adm_ctrl_target->set_desired_force(target.bfrc_desired);

//   math::Matrix4d xform_ctc = contact->get_body2()->get_xform(tforms)
//                              * contact->get_frame2()->get_tform();
//   xframe_desired->set_xform(xform_ctc);
//   m_ctrl_targets.push_back(std::make_pair(adm_ctrl_target, xframe_desired));
// }

// //==============================================================================
// kdl::Grasp::ConstPtr SlidingAction::get_grasp() const {
//   return m_grasp;
// }
