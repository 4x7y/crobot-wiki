
#include "crobot/ekf/state_obs.hpp"

using namespace crobot;
using namespace ekf;

//==============================================================================
StateObs::StateObs(
    kdl::DynamicGrasp::ConstPtr grasp, kdl::ConstRigidBodyTreePtr robot)
  : m_grasp(grasp->clone()), m_robot(robot) {

  std::string obj_name = grasp->get_body()->get_name();
  m_obj                = m_robot->get_body(obj_name);
  m_obj_qpos_addr      = m_obj->get_support_qpos_addr();
  m_obj_qvel_addr      = m_obj->get_support_qvel_addr();

  std::sort(m_obj_qpos_addr.begin(), m_obj_qpos_addr.end());
  std::sort(m_obj_qvel_addr.begin(), m_obj_qvel_addr.end());

  std::vector<math::Vector3d> forces;
  for (auto& contact : m_grasp->get_dynamic_contacts()) {
    forces.push_back(contact->get_xfrc());
  }

  // initialize the state
  update_sensor_state(
      m_robot->make_zero_qpos(), m_robot->make_zero_qvel(), forces);

  std::vector<math::Vector2d> object_params, finger_params;
  for (const auto& contact : m_grasp->get_dynamic_contacts()) {
    object_params.push_back(contact->get_contact_points().first);
    finger_params.push_back(contact->get_contact_points().second);
  }
  math::VectorXd pose = math::VectorXd::Zero(7);
  pose(3)             = 1;
  update_obj_state(pose, math::Vector6d::Zero(), object_params, finger_params);
}

//==============================================================================
void StateObs::update_sensor_state(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    std::vector<math::Vector3d> forces) {
  CROBOT_ASSERT_ERROR(
      qpos.size() == m_robot->get_qpos_dim(), "qpos size mismatch");
  CROBOT_ASSERT_ERROR(
      qvel.size() == m_robot->get_qvel_dim(), "qvel size mismatch");

  CROBOT_ASSERT_ERROR(
      forces.size() == m_grasp->get_dynamic_contacts().size(),
      "forces size mismatch");

  m_qpos   = qpos;
  m_qvel   = qvel;
  m_tforms = m_robot->forward_kinematics(qpos);
  m_xfrc   = forces;
}

//==============================================================================
void StateObs::update_obj_state(
    const math::VectorXd& obj_pos,
    const math::Vector6d& obj_bvel,
    std::vector<math::Vector2d> obj_surf_params,
    std::vector<math::Vector2d> fin_surf_params) {
  CROBOT_ASSERT_ERROR(obj_pos.size() == 7, "obj_pos size mismatch");
  CROBOT_ASSERT_ERROR(obj_bvel.size() == 6, "obj_vel size mismatch");
  CROBOT_ASSERT_ERROR(
      obj_surf_params.size() == m_grasp->get_dynamic_contacts().size(),
      "obj_surf_params size mismatch");

  CROBOT_ASSERT_ERROR(
      fin_surf_params.size() == m_grasp->get_dynamic_contacts().size(),
      "fin_surf_params size mismatch");

  m_ctc_vels.clear();
  m_A.clear();
  m_B.clear();
  m_C.clear();
  m_D.clear();

  m_obj_pos  = obj_pos;
  m_obj_bvel = obj_bvel;

  m_qpos.segment<3>(m_obj_qpos_addr[0]) = obj_pos.segment<3>(0);
  m_qpos.segment<4>(m_obj_qpos_addr[0] + 3)
      = obj_pos.segment<4>(3).normalized();
  m_qvel.segment<6>(m_obj_qvel_addr[0]) = obj_bvel;

  // std::cout << "m_qvel = " << m_qvel.transpose() << std::endl;
  // std::cout << "obj_bvel = " << obj_bvel.transpose() << std::endl;

  m_tforms = m_robot->forward_kinematics(m_qpos);

  std::vector<kdl::BodyRollingContact::Ptr> contacts
      = m_grasp->get_dynamic_contacts();

  for (size_t i = 0; i < obj_surf_params.size(); ++i) {
    contacts[i]->set_contact_points(obj_surf_params[i], fin_surf_params[i]);

    math::SurfacePatchSwitch::ConstPtr patch_obj = contacts[i]->get_surface1();
    math::SurfacePatchSwitch::ConstPtr patch_fin = contacts[i]->get_surface2();
    m_A.push_back(patch_obj->jacb(obj_surf_params[i]));
    m_B.push_back(patch_fin->jacb(fin_surf_params[i]));
    m_C.push_back(patch_obj->normal_jacb(obj_surf_params[i]));
    m_D.push_back(patch_fin->normal_jacb(fin_surf_params[i]));

    m_ctc_vels.push_back(contacts[i]->get_moving_dir(m_tforms, m_qvel));
  }
}

//==============================================================================
std::vector<std::pair<math::Vector2d, math::Vector2d>> StateObs::get_ctc_vels()
    const {
  return m_ctc_vels;
}

//==============================================================================
std::vector<math::Vector3d> StateObs::get_xfrc() const {
  return m_xfrc;
}

//==============================================================================
std::tuple<math::MatrixXd, math::MatrixXd, math::MatrixXd> StateObs::get_dsv()
    const {
  double pinv_eps = 1e-5;

  int num_contacts = m_grasp->get_num_contacts();

  math::Matrix4d xform_obj = m_obj->get_xform(m_tforms);
  math::Quaternion quat_obj(
      m_obj_pos(3), m_obj_pos(4), m_obj_pos(5), m_obj_pos(6));
  math::Matrix3d rotm_obj = xform_obj.tform2rotm();

  auto contacts = m_grasp->get_dynamic_contacts();

  math::Vector3d bvelr_obj = m_obj_bvel.tail<3>();

  math::MatrixXd dsv_dquat  = math::MatrixXd::Zero(4 * num_contacts, 4);
  math::MatrixXd dsv_domega = math::MatrixXd::Zero(4 * num_contacts, 3);
  math::MatrixXd dsv_dctc
      = math::MatrixXd::Zero(4 * num_contacts, 4 * num_contacts);

  for (int i = 0; i < num_contacts; ++i) {
    kdl::BodyRollingContact::Ptr contact = contacts[i];
    math::Vector2d contact_obj           = contact->get_contact_points().first;
    math::Vector2d contact_fin           = contact->get_contact_points().second;
    math::SurfacePatchSwitch::ConstPtr patch_obj = contact->get_surface1();
    math::SurfacePatchSwitch::ConstPtr patch_fin = contact->get_surface2();

    math::Vector2d xi_dot  = m_ctc_vels[i].first;
    math::Vector2d eta_dot = m_ctc_vels[i].second;

    math::Vector3d norm_obj = patch_obj->normal(contact_obj);
    (void)patch_fin; // Suppress unused warning

    math::Matrix4d xform_fin = contact->get_body2()->get_xform(m_tforms);
    math::Matrix3d rotm_fin  = xform_fin.tform2rotm();

    math::Jacobian jacb_fin = contact->get_body2()->get_jacb(m_tforms);
    math::Vector3d velr_fin
        = contact->get_body2()->get_xvel(jacb_fin, m_qvel).head<3>();

    math::Matrix32 RA = rotm_obj * m_A[i];
    math::Matrix32 RC = rotm_obj * m_C[i];
    math::Matrix32 RB = rotm_fin * m_B[i];
    math::Matrix32 RD = rotm_fin * m_D[i];

    math::Matrix6X A    = math::Matrix6X::Zero(6, 4);
    A.block<3, 2>(0, 0) = RA;
    A.block<3, 2>(0, 2) = -RB;
    A.block<3, 2>(3, 0) = RC;
    A.block<3, 2>(3, 2) = RD;

    // d(obj quaternion)
    math::Matrix6X b1    = math::Matrix6X::Zero(6, 4);
    b1.block<3, 4>(0, 0) = -quat_obj.diff_rotvec_coeffs_wxyz(m_A[i] * xi_dot);
    b1.block<3, 4>(3, 0)
        = -quat_obj.diff_rotvec_coeffs_wxyz(m_C[i] * xi_dot)
          - (rotm_obj * bvelr_obj).skew()
                * (quat_obj.diff_rotvec_coeffs_wxyz(norm_obj))
          + (rotm_obj * norm_obj).skew()
                * (quat_obj.diff_rotvec_coeffs_wxyz(bvelr_obj));

    // b1.block<3, 4>(0, 0)
    //     = -quat_obj.nonunit_diff_rotvec_coeffs(m_A[i] * xi_dot);
    // b1.block<3, 4>(3, 0)
    //     = -quat_obj.nonunit_diff_rotvec_coeffs(m_C[i] * xi_dot)
    //       - (rotm_obj * bvelr_obj).skew()
    //             * (quat_obj.nonunit_diff_rotvec_coeffs(norm_obj))
    //       + (rotm_obj * norm_obj).skew()
    //             * (quat_obj.nonunit_diff_rotvec_coeffs(bvelr_obj));

    dsv_dquat.block<4, 4>(4 * i, 0)
        = math::calculate_jacb_inv(A, pinv_eps) * b1;

    // d(obj angular velocity)
    math::Matrix6X b2    = math::Matrix6X::Zero(6, 3);
    b2.block<3, 3>(3, 0) = (rotm_obj * norm_obj).skew() * rotm_obj;

    dsv_domega.block<4, 3>(4 * i, 0)
        = math::calculate_jacb_inv(A, pinv_eps) * b2;

    // d(ctc position on obj)
    math::Matrix6X b3_obj = math::Matrix6X::Zero(6, 2);

    math::Matrix6X hess_obj      = patch_obj->hess(contact_obj);
    math::Matrix32 hess_obj_x    = math::Matrix32::Zero();
    hess_obj_x.block<3, 1>(0, 0) = hess_obj.topRows(3) * xi_dot;
    hess_obj_x.block<3, 1>(0, 1) = hess_obj.bottomRows(3) * xi_dot;

    math::Matrix6X norm_hess_obj      = patch_obj->normal_hess(contact_obj);
    math::Matrix32 norm_hess_obj_x    = math::Matrix32::Zero();
    norm_hess_obj_x.block<3, 1>(0, 0) = norm_hess_obj.topRows(3) * xi_dot;
    norm_hess_obj_x.block<3, 1>(0, 1) = norm_hess_obj.bottomRows(3) * xi_dot;

    b3_obj.block<3, 2>(0, 0) = -rotm_obj * hess_obj_x;
    b3_obj.block<3, 2>(3, 0)
        = -rotm_obj * norm_hess_obj_x
          - (rotm_obj * bvelr_obj).skew() * rotm_obj * m_C[i];

    dsv_dctc.block<4, 2>(4 * i, 4 * i)
        = math::calculate_jacb_inv(A, pinv_eps) * b3_obj;

    // d(ctc position on fin)
    math::Matrix6X b3_fin = math::Matrix6X::Zero(6, 2);

    math::Matrix6X hess_fin      = patch_fin->hess(contact_fin);
    math::Matrix32 hess_fin_x    = math::Matrix32::Zero();
    hess_fin_x.block<3, 1>(0, 0) = hess_fin.topRows(3) * eta_dot;
    hess_fin_x.block<3, 1>(0, 1) = hess_fin.bottomRows(3) * eta_dot;

    math::Matrix6X norm_hess_fin      = patch_fin->normal_hess(contact_fin);
    math::Matrix32 norm_hess_fin_x    = math::Matrix32::Zero();
    norm_hess_fin_x.block<3, 1>(0, 0) = norm_hess_fin.topRows(3) * eta_dot;
    norm_hess_fin_x.block<3, 1>(0, 1) = norm_hess_fin.bottomRows(3) * eta_dot;

    b3_fin.block<3, 2>(0, 0) = rotm_fin * hess_fin_x;
    b3_fin.block<3, 2>(3, 0)
        = -rotm_fin * norm_hess_fin_x - velr_fin.skew() * rotm_fin * m_D[i];

    dsv_dctc.block<4, 2>(4 * i, 4 * i + 2)
        = math::calculate_jacb_inv(A, pinv_eps) * b3_fin;
  }

  return std::make_tuple(dsv_dquat, dsv_domega, dsv_dctc);
}

//==============================================================================
kdl::DynamicGrasp::ConstPtr StateObs::get_grasp() const {
  return m_grasp;
}

//==============================================================================
std::vector<math::Matrix4d> StateObs::get_tforms() const {
  return m_tforms;
}
//==============================================================================
// get d(contact constraints) 6 DOF
// d(obj pos) 6n x 3
// d(obj quat) 6n x 4
// d(surf_params) 6n x 4n
std::tuple<math::MatrixXd, math::MatrixXd, math::MatrixXd>
StateObs::get_dctc_cons() const {

  int num_contacts = m_grasp->get_num_contacts();

  math::Quaternion quat_obj(
      m_obj_pos(3), m_obj_pos(4), m_obj_pos(5), m_obj_pos(6));
  math::Matrix3d xmat_obj = quat_obj.normalized().toRotationMatrix();
  // math::Quaternion quat_obj(
  //     m_qpos(m_obj_qpos_addr[0] + 3),
  //     m_qpos(m_obj_qpos_addr[0] + 4),
  //     m_qpos(m_obj_qpos_addr[0] + 5),
  //     m_qpos(m_obj_qpos_addr[0] + 6));

  auto contacts = m_grasp->get_dynamic_contacts();

  math::MatrixXd dobj_pos_all  = math::MatrixXd::Zero(5 * num_contacts, 3);
  math::MatrixXd dobj_quat_all = math::MatrixXd::Zero(5 * num_contacts, 4);
  math::MatrixXd dobj_surf_params_all
      = math::MatrixXd::Zero(5 * num_contacts, 4 * num_contacts);

  math::Matrix34 J_r
      = quat_obj.normalized().diff_omega_coeffs() * quat_obj.unit_deriv();

  for (int i = 0; i < num_contacts; i++) {
    kdl::BodyRollingContact::Ptr contact = contacts[i];
    math::Matrix3d xmat_fin
        = contact->get_body2()->get_xform(m_tforms).tform2rotm();

    math::Vector2d contact_obj = contact->get_contact_points().first;
    math::Vector2d contact_fin = contact->get_contact_points().second;
    math::SurfacePatchSwitch::ConstPtr patch_obj = contact->get_surface1();
    math::SurfacePatchSwitch::ConstPtr patch_fin = contact->get_surface2();

    math::Vector3d ctc_xpos_obj = xmat_obj * patch_obj->evaluate(contact_obj);
    math::Vector3d xtan_u_obj = xmat_obj * patch_obj->jacb(contact_obj).col(0);
    math::Vector3d xtan_v_obj = xmat_obj * patch_obj->jacb(contact_obj).col(1);
    ;
    math::array2<math::Vector3d> derivs
        = patch_obj->derivatives(2, contact_obj);

    math::Vector3d bnorm_fin     = patch_fin->normal(contact_fin);
    math::Vector3d ctc_xnorm_fin = xmat_fin * bnorm_fin;

    math::Matrix5X dobj_pos    = math::Matrix5X::Zero(5, 3);
    dobj_pos.block<3, 3>(0, 0) = math::Matrix3X::Identity(3, 3);

    math::Matrix5X dobj_quat    = math::Matrix5X::Zero(5, 4);
    dobj_quat.block<3, 4>(0, 0) = -ctc_xpos_obj.skew() * J_r;
    dobj_quat.block<1, 4>(3, 0)
        = -ctc_xnorm_fin.transpose() * (xtan_u_obj.skew() * J_r);
    dobj_quat.block<1, 4>(4, 0)
        = -ctc_xnorm_fin.transpose() * (xtan_v_obj.skew() * J_r);

    // dobj_quat.block<3, 4>(0, 0) =
    // quat_obj.nonunit_diff_rotvec_coeffs(bpos_obj); dobj_quat.block<3, 4>(3,
    // 0) = quat_obj.nonunit_diff_rotvec_coeffs(norm_obj);

    math::Matrix5X dobj_surf_params    = math::Matrix5X::Zero(5, 4);
    dobj_surf_params.block<3, 2>(0, 0) = xmat_obj * m_A[i];
    dobj_surf_params.block<3, 2>(0, 2) = -xmat_fin * m_B[i];

    math::Matrix32 dsu_obj;
    dsu_obj.block<3, 1>(0, 0) = derivs(2, 0);
    dsu_obj.block<3, 1>(0, 1) = derivs(1, 1);

    math::Matrix32 dsv_obj;
    dsv_obj.block<3, 1>(0, 0) = derivs(1, 1);
    dsv_obj.block<3, 1>(0, 1) = derivs(0, 2);

    dobj_surf_params.block<1, 2>(3, 0)
        = ctc_xnorm_fin.transpose() * (xmat_obj * dsu_obj);
    dobj_surf_params.block<1, 2>(4, 0)
        = ctc_xnorm_fin.transpose() * (xmat_obj * dsv_obj);

    dobj_surf_params.block<1, 2>(3, 2)
        = xtan_u_obj.transpose() * (xmat_fin * m_D[i]);
    dobj_surf_params.block<1, 2>(4, 2)
        = xtan_v_obj.transpose() * (xmat_fin * m_D[i]);

    dobj_pos_all.block<5, 3>(5 * i, 0)             = dobj_pos;
    dobj_quat_all.block<5, 4>(5 * i, 0)            = dobj_quat;
    dobj_surf_params_all.block<5, 4>(5 * i, 4 * i) = dobj_surf_params;
  }

  return std::make_tuple(dobj_pos_all, dobj_quat_all, dobj_surf_params_all);
}
