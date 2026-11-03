
#include "crobot/ekf/pose_tracking_system.hpp"

using namespace crobot;
using namespace ekf;

//==============================================================================
PoseTrackingSystem::PoseTrackingSystem(
    const math::VectorXd& x0,
    const math::VectorXd& u0,
    const math::MatrixXd& P0,
    const math::MatrixXd& Q,
    const math::MatrixXd& R,
    const math::MatrixXd& P0_reset,
    int const_dim,
    const std::string& c_eps_file,
    ObjDynamics::Ptr obj_dynamics,
    StateObs::Ptr robot_state)
  : KalmanFilterSystem(x0, u0, P0, Q, R, P0_reset, const_dim),
    m_obj_dynamics(obj_dynamics),
    m_robot_state(robot_state) {

  YAML::Node c_eps_node = YAML::LoadFile(c_eps_file);

  double unit_eps = c_eps_node["unit_eps"].as<double>();
  double pos_eps  = c_eps_node["pos_eps"].as<double>();
  double ori_eps  = c_eps_node["quat_eps"].as<double>();

  m_c_eps    = math::VectorXd::Zero(get_const_dim());
  m_c_eps[0] = unit_eps;
  m_c_eps[1] = unit_eps;

  int counter = 2;
  for (int i = 0; i < m_robot_state->get_grasp()->get_num_contacts(); i++) {
    m_c_eps[counter]     = pos_eps;
    m_c_eps[counter + 1] = pos_eps;
    m_c_eps[counter + 2] = pos_eps;
    m_c_eps[counter + 3] = ori_eps;
    m_c_eps[counter + 4] = ori_eps;
    counter += 5;
  }
}

//==============================================================================
PoseTrackingSystem::PoseTrackingSystem(
    const math::VectorXd& x0,
    const math::VectorXd& u0,
    const math::MatrixXd& P0,
    const math::MatrixXd& Q,
    const math::MatrixXd& R,
    const math::MatrixXd& P0_reset,
    int const_dim,
    YAML::Node c_eps_node,
    ObjDynamics::Ptr obj_dynamics,
    StateObs::Ptr robot_state)
  : KalmanFilterSystem(x0, u0, P0, Q, R, P0_reset, const_dim),
    m_obj_dynamics(obj_dynamics),
    m_robot_state(robot_state) {

  double unit_eps = c_eps_node["unit_eps"].as<double>();
  double pos_eps  = c_eps_node["pos_eps"].as<double>();
  double ori_eps  = c_eps_node["quat_eps"].as<double>();

  m_c_eps    = math::VectorXd::Zero(get_const_dim());
  m_c_eps[0] = unit_eps;
  m_c_eps[1] = unit_eps;

  int counter = 2;
  for (int i = 0; i < m_robot_state->get_grasp()->get_num_contacts(); i++) {
    m_c_eps[counter]     = pos_eps;
    m_c_eps[counter + 1] = pos_eps;
    m_c_eps[counter + 2] = pos_eps;
    m_c_eps[counter + 3] = ori_eps;
    m_c_eps[counter + 4] = ori_eps;
    counter += 5;
  }
}

//==============================================================================
// return x_dot
math::VectorXd PoseTrackingSystem::f(
    const math::VectorXd& x,
    const math::VectorXd& u [[maybe_unused]],
    const math::VectorXd& w) const {

  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;

  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  // update obj state
  m_robot_state->update_obj_state(pos, bvel, obj_surf_params, fin_surf_params);

  // forces
  std::vector<math::Vector3d> forces = m_robot_state->get_xfrc();

  math::VectorXd x_dot = math::VectorXd::Zero(x.size());
  x_dot.segment<3>(0)  = bvel.head<3>(); // velp

  math::Vector3d bvelr = bvel.tail<3>();
  math::Quaternion bvelr_quat(0, bvelr(0), bvelr(1), bvelr(2));
  math::Quaternion quat(pos(3), pos(4), pos(5), pos(6));
  x_dot.segment<4>(3) = 0.5 * (quat * bvelr_quat).as_vector(); // quat_dot

  // x_dot.segment<6>(7)
  //     = m_obj_dynamics->get_acc(pos, bvel, obj_surf_params, forces);

  x_dot.segment<6>(7) = math::Vector6d::Zero();

  std::vector<std::pair<math::Vector2d, math::Vector2d>> ctc_vels
      = m_robot_state->get_ctc_vels();

  for (size_t i = 0; i < obj_surf_params.size(); i++) {
    x_dot.segment<2>(13 + 4 * i) = ctc_vels[i].first;
    x_dot.segment<2>(15 + 4 * i) = ctc_vels[i].second;
  }

  return x_dot + w;
}

//==============================================================================
math::SparseJacobian PoseTrackingSystem::get_dfdx(
    const math::VectorXd& x,
    const math::VectorXd& u [[maybe_unused]],
    const math::VectorXd& w [[maybe_unused]]) const {

  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;
  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  // update obj state
  m_robot_state->update_obj_state(pos, bvel, obj_surf_params, fin_surf_params);

  // forces
  std::vector<math::Vector3d> forces = m_robot_state->get_xfrc();

  // result
  math::SparseJacobian dfdx = math::SparseJacobian(x.size(), x.size());

  int counter = 0;

  // d(velp)
  dfdx.coeffRef(0, 7) = 1;
  dfdx.coeffRef(1, 8) = 1;
  dfdx.coeffRef(2, 9) = 1;

  counter += 3;

  // d(quat_dot)/d(quat)
  math::Vector3d bvelr = bvel.tail<3>();
  math::Quaternion bvelr_quat(0, bvelr(0), bvelr(1), bvelr(2));
  math::Quaternion quat(pos(3), pos(4), pos(5), pos(6));
  math::Matrix4d dquat_dot_dquat = 0.5 * bvelr_quat.diff_other_times_coeffs();

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      dfdx.coeffRef(i + counter, j + 3) = dquat_dot_dquat(i, j);
    }
  }

  // // d(quat_dot)/d(velr)
  math::Matrix43 dquat_dot_dvelr
      = 0.5 * quat.diff_times_other_coeffs().block<4, 3>(0, 1);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      dfdx.coeffRef(i + counter, j + 10) = dquat_dot_dvelr(i, j);
    }
  }

  counter += 4;

  // d(acc)/d(pos, quat)
  // math::Matrix6X dadp
  //     = m_obj_dynamics->get_dadp(pos, bvel, obj_surf_params, forces);
  // for (int i = 0; i < 6; i++) {
  //   for (int j = 0; j < 7; j++) {
  //     dfdx.coeffRef(i + counter, j) = dadp(i, j);
  //   }
  // }

  // // d(acc)/d(velp)
  // math::Matrix6d dadv
  //     = m_obj_dynamics->get_dadv(pos, bvel, obj_surf_params, forces);
  // for (int i = 0; i < 6; i++) {
  //   for (int j = 0; j < 6; j++) {
  //     dfdx.coeffRef(i + counter, j + 7) = dadv(i, j);
  //   }
  // }

  // // d(acc)/d(obj_surf_params)
  // math::Matrix6X dadc
  //     = m_obj_dynamics->get_dadc(pos, bvel, obj_surf_params, forces);

  // for (int i = 0; i < 6; i++) {
  //   for (int j = 0; j < obj_surf_params.size(); j++) {
  //     dfdx.coeffRef(i + counter, 13 + 4 * j) = dadc(i, 2 * j);
  //     dfdx.coeffRef(i + counter, 14 + 4 * j) = dadc(i, 2 * j + 1);
  //   }
  // }

  counter += 6;

  math::MatrixXd dsv_dquat, dsv_domega, dsv_dctc;
  std::tie(dsv_dquat, dsv_domega, dsv_dctc) = m_robot_state->get_dsv();

  // d(surface param vels)/d(obj quaternion)
  for (size_t i = 0; i < obj_surf_params.size(); i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        dfdx.coeffRef(4 * i + counter + j, k + 3) = dsv_dquat(4 * i + j, k);
      }
    }
  }

  // d(surface param vels)/d(obj angular velocity)
  for (size_t i = 0; i < obj_surf_params.size(); i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 3; k++) {
        dfdx.coeffRef(4 * i + counter + j, k + 10) = dsv_domega(4 * i + j, k);
      }
    }
  }

  // d(surface param vels)/d(ctc position on obj)
  for (size_t i = 0; i < obj_surf_params.size(); i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        dfdx.coeffRef(4 * i + counter + j, 4 * i + counter + k)
            = dsv_dctc(4 * i + j, 4 * i + k);
      }
    }
  }

  return dfdx;
}

//==============================================================================
math::SparseJacobian PoseTrackingSystem::get_dfdw(
    const math::VectorXd& x [[maybe_unused]],
    const math::VectorXd& u [[maybe_unused]],
    const math::VectorXd& w [[maybe_unused]]) const {

  math::SparseJacobian dfdw = math::SparseJacobian(get_xdim(), get_wdim());
  for (int i = 0; i < get_wdim(); i++) {
    dfdw.coeffRef(i, i) = 1;
  }
  return dfdw;
}

//==============================================================================
math::VectorXd PoseTrackingSystem::h(
    const math::VectorXd& x, const math::VectorXd& v) const {

  CROBOT_ASSERT_ERROR(v.size() == get_ydim(), "v.size() != get_ydim()");
  CROBOT_ASSERT_ERROR(x.size() == get_xdim(), "x.size() != get_xdim()");

  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;
  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  math::Vector3d xpos_obj = pos.head<3>();
  math::Quaternion xquat_obj(pos(3), pos(4), pos(5), pos(6));

  // std::cout << "xquat_obj: " << xquat_obj.as_vector().transpose() <<
  // std::endl;

  // xquat_obj.normalize();

  math::Vector3d xpos_cam = cam_pose.head<3>();
  math::Quaternion xquat_cam(
      cam_pose(3), cam_pose(4), cam_pose(5), cam_pose(6));
  // xquat_cam.normalize();

  math::VectorXd y = math::VectorXd::Zero(get_ydim());

  int counter = 0;

  y.segment<3>(counter) = xquat_cam.conjugate() * (xpos_obj - xpos_cam);
  counter += 3;

  y.segment<4>(counter) = (xquat_cam.conjugate() * xquat_obj).as_vector();

  return y + v;
}

//==============================================================================
math::SparseJacobian PoseTrackingSystem::get_dhdx(
    const math::VectorXd& x, const math::VectorXd& v [[maybe_unused]]) const {
  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;
  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  math::Vector3d xpos_obj = pos.head<3>();
  math::Quaternion xquat_obj(pos(3), pos(4), pos(5), pos(6));
  // math::Matrix3d xrotm_obj = xquat_obj.toRotationMatrix();
  double q0_obj         = xquat_obj.w();
  math::Vector3d qv_obj = xquat_obj.vec();

  math::Vector3d xpos_cam = cam_pose.head<3>();
  math::Quaternion xquat_cam(
      cam_pose(3), cam_pose(4), cam_pose(5), cam_pose(6));

  // xquat_cam.normalize();

  math::Matrix3d xrotm_cam = xquat_cam.toRotationMatrix();
  (void)xquat_cam; // Suppress unused warning

  // update obj state
  // m_robot_state->update_obj_state(pos, bvel, obj_surf_params,
  // fin_surf_params);

  // result
  math::SparseJacobian dhdx = math::SparseJacobian(get_ydim(), get_xdim());
  int ind_cam_pose          = get_xdim() - 7;

  int counter = 0;

  // d(cpos_obj) / d(pos_obj)
  math::Matrix3d dcpos_dxpos_obj = xrotm_cam.transpose();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dhdx.coeffRef(i + counter, j) = dcpos_dxpos_obj(i, j);
    }
  }

  // d(cpos_obj) / d(pos_cam)
  math::Matrix3d dcpos_dxpos_cam = -xrotm_cam.transpose();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dhdx.coeffRef(i + counter, j + ind_cam_pose) = dcpos_dxpos_cam(i, j);
    }
  }

  // d(cpos_obj) / d(quat_cam)
  math::Matrix34 dcpos_dxquat_cam = xrotm_cam.transpose()
                                    * (xpos_obj - xpos_cam).skew()
                                    * xquat_cam.diff_omega_coeffs();
  // = xquat_cam.diff_conjugate_rotvec_coeffs(xpos_obj - xpos_cam);
  // = xquat_cam.nonunit_diff_conjugate_rotvec_coeffs(xpos_obj - xpos_cam);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      dhdx.coeffRef(i + counter, j + ind_cam_pose + 3) = dcpos_dxquat_cam(i, j);
    }
  }

  counter += 3;

  // d(cquat_obj) / d(quat_obj)
  math::Matrix4d dcquat_dquat_obj
      = xquat_cam.conjugate().diff_times_other_coeffs();

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      dhdx.coeffRef(i + counter, j + 3) = dcquat_dquat_obj(i, j);
    }
  }

  // d(cquat_obj) / d(quat_cam)
  math::Matrix4d dcquat_dquat_cam    = math::Matrix4d::Zero();
  dcquat_dquat_cam(0, 0)             = q0_obj;
  dcquat_dquat_cam.block<1, 3>(0, 1) = qv_obj.transpose();
  dcquat_dquat_cam.block<3, 1>(1, 0) = qv_obj;
  dcquat_dquat_cam.block<3, 3>(1, 1)
      = -q0_obj * math::Matrix3d::Identity() + qv_obj.skew();

  // std::cout << "qv_obj: " << qv_obj.transpose() << std::endl;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      dhdx.coeffRef(i + counter, j + ind_cam_pose + 3) = dcquat_dquat_cam(i, j);
    }
  }

  return dhdx;
}

//==============================================================================
math::SparseJacobian PoseTrackingSystem::get_dhdv(
    const math::VectorXd& x [[maybe_unused]],
    const math::VectorXd& v [[maybe_unused]]) const {
  math::SparseJacobian dhdv = math::SparseJacobian(get_ydim(), get_vdim());
  for (int i = 0; i < get_vdim(); i++) {
    dhdv.coeffRef(i, i) = 1;
  }
  return dhdv;
}

//==============================================================================
std::tuple<
    math::VectorXd,
    math::Vector6d,
    std::vector<math::Vector2d>,
    std::vector<math::Vector2d>,
    math::VectorXd>
PoseTrackingSystem::unpack_x(const math::VectorXd& x) const {

  // x = [obj_pos7, obj_vel6, (obj_surf_param 2 , fin_surf_param 2) *
  // num_contact, cam_pose7]
  CROBOT_ASSERT_ERROR(
      x.size() == 7 + 6 + 4 * m_obj_dynamics->get_num_contacts() + 7,
      "x.size() is incorrect");

  int counter = 0;

  math::VectorXd pos = x.segment<7>(counter);
  counter += 7;

  math::Vector6d vel = x.segment<6>(counter);
  counter += 6;

  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;

  for (int i = 0; i < m_obj_dynamics->get_num_contacts(); i++) {
    obj_surf_params.push_back(x.segment<2>(counter));
    fin_surf_params.push_back(x.segment<2>(counter + 2));
    counter += 4;
  }

  math::VectorXd cam_pose = x.segment<7>(counter);

  return std::make_tuple(pos, vel, obj_surf_params, fin_surf_params, cam_pose);
}
//==============================================================================
math::VectorXd PoseTrackingSystem::c(const math::VectorXd& x) const {

  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;
  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  // update obj state
  m_robot_state->update_obj_state(pos, bvel, obj_surf_params, fin_surf_params);

  int counter = 0;

  math::VectorXd c = math::VectorXd::Zero(
      get_const_dim() - 2); // quaternion constraints not included

  std::vector<math::Matrix4d> tforms = m_robot_state->get_tforms();

  math::Matrix3d xmat_obj
      = math::Quaternion(pos(3), pos(4), pos(5), pos(6)).toRotationMatrix();

  // contact constraints
  kdl::DynamicGrasp::ConstPtr grasp = m_robot_state->get_grasp();
  for (auto contact : grasp->get_dynamic_contacts()) {

    math::Vector2d contact_obj = contact->get_contact_points().first;

    math::Vector3d ctc_xpos_obj = contact->get_xpos1(tforms);

    math::Matrix3d ctc_xmat_fin  = contact->get_xmat2(tforms);
    math::Vector3d ctc_xpos_fin  = contact->get_xpos2(tforms);
    math::Vector3d ctc_xnorm_fin = ctc_xmat_fin.col(2);

    // orientation of the normal should be in the opposite direction
    math::Vector3d xtan_u_obj
        = xmat_obj * contact->get_surface1()->jacb(contact_obj).col(0);
    math::Vector3d xtan_v_obj
        = xmat_obj * contact->get_surface1()->jacb(contact_obj).col(1);

    c.segment<3>(counter) = ctc_xpos_obj - ctc_xpos_fin;
    counter += 3;

    c.segment(counter, 1)     = xtan_u_obj.transpose() * ctc_xnorm_fin;
    c.segment(counter + 1, 1) = xtan_v_obj.transpose() * ctc_xnorm_fin;
    counter += 2;
  }

  // quaternion constraints
  // c(counter) = pos.segment<4>(3).dot(pos.segment<4>(3)) - 1;
  // counter += 1;

  // c(counter) = cam_pose.segment<4>(3).dot(cam_pose.segment<4>(3)) - 1;

  return c;
}

//==============================================================================
math::SparseJacobian PoseTrackingSystem::get_dcdx(
    const math::VectorXd& x) const {

  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;
  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  math::Quaternion xquat_obj(pos(3), pos(4), pos(5), pos(6));

  math::Quaternion xquat_cam(
      cam_pose(3), cam_pose(4), cam_pose(5), cam_pose(6));

  // update obj state
  m_robot_state->update_obj_state(pos, bvel, obj_surf_params, fin_surf_params);

  // result
  math::SparseJacobian dcdx = math::SparseJacobian(
      get_const_dim() - 2, get_xdim()); // quaternion constraints not included

  int counter = 0;

  // contact constraints
  math::MatrixXd dobj_pos, dobj_quat, dobj_surf_params;
  std::tie(dobj_pos, dobj_quat, dobj_surf_params)
      = m_robot_state->get_dctc_cons();

  for (int i = 0; i < dobj_pos.rows(); i++) {
    for (int j = 0; j < dobj_pos.cols(); j++) {
      dcdx.coeffRef(i + counter, j) = dobj_pos(i, j);
    }

    for (int j = 0; j < dobj_quat.cols(); j++) {
      dcdx.coeffRef(i + counter, j + 3) = dobj_quat(i, j);
    }

    for (int j = 0; j < dobj_surf_params.cols(); j++) {
      dcdx.coeffRef(i + counter, j + 13) = dobj_surf_params(i, j);
    }
  }

  // counter += dobj_pos.rows();

  // // quaternion constraints
  // math::Matrix1X dquatc_dquat_obj = 2 * xquat_obj.as_vector();
  // for (int i = 0; i < dquatc_dquat_obj.cols(); i++) {
  //   dcdx.coeffRef(counter, i + 3) = dquatc_dquat_obj(i);
  // }

  // math::Matrix1X dquatc_dquat_cam = 2 * xquat_cam.as_vector();
  // for (int i = 0; i < dquatc_dquat_cam.cols(); i++) {
  //   dcdx.coeffRef(counter + 1, i + ind_cam_pose + 3) = dquatc_dquat_cam(i);
  // }

  return dcdx;
}

//==============================================================================
std::tuple<
    std::vector<math::MatrixXd>,
    std::vector<math::VectorXd>,
    std::vector<double>>
PoseTrackingSystem::get_constraints(const math::VectorXd& x) const {

  std::vector<math::MatrixXd> A_all;
  std::vector<math::VectorXd> b_all;
  std::vector<double> c_all;

  // unit obj quaternion
  math::MatrixXd A1 = math::MatrixXd::Zero(get_xdim(), get_xdim());
  for (int i = 3; i < 7; i++) {
    A1(i, i) = 1;
  }
  A_all.push_back(A1);
  b_all.push_back(math::VectorXd::Zero(get_xdim()));
  c_all.push_back(-1);

  // unit cam quaternion
  math::MatrixXd A2 = math::MatrixXd::Zero(get_xdim(), get_xdim());
  for (int i = get_xdim() - 4; i < get_xdim(); i++) {
    A2(i, i) = 1;
  }
  A_all.push_back(A2);
  b_all.push_back(math::VectorXd::Zero(get_xdim()));
  c_all.push_back(-1);

  // contact constraints
  math::MatrixXd A0           = math::MatrixXd::Zero(get_xdim(), get_xdim());
  math::MatrixXd dcdx         = get_dcdx(x);
  math::VectorXd constratints = c(x);

  math::VectorXd c = -(dcdx * x - constratints);

  CROBOT_ASSERT_ERROR(c.size() == dcdx.rows(), "c.size() != dcdx.rows()");

  for (int i = 0; i < c.size(); i++) {
    A_all.push_back(A0);
    b_all.push_back(dcdx.row(i).transpose());
    c_all.push_back(c(i));
  }

  CROBOT_ASSERT_ERROR(
      static_cast<int>(A_all.size()) == get_const_dim(),
      "A_all.size() = %ld, get_const_dim() = %d",
      A_all.size(),
      get_const_dim());

  return std::make_tuple(A_all, b_all, c_all);
}

//==============================================================================
void PoseTrackingSystem::update_sensor_state(
    const math::VectorXd& qpos,
    const math::VectorXd& qvel,
    std::vector<math::Vector3d> forces) {
  m_robot_state->update_sensor_state(qpos, qvel, forces);
}

//==============================================================================

void PoseTrackingSystem::update_obj_state(const math::VectorXd& x) {

  math::VectorXd pos;
  math::Vector6d bvel;
  std::vector<math::Vector2d> obj_surf_params;
  std::vector<math::Vector2d> fin_surf_params;
  math::VectorXd cam_pose;
  std::tie(pos, bvel, obj_surf_params, fin_surf_params, cam_pose) = unpack_x(x);

  m_robot_state->update_obj_state(pos, bvel, obj_surf_params, fin_surf_params);
}

//==============================================================================
std::tuple<math::MatrixXd, math::VectorXd>
PoseTrackingSystem::get_linear_constraints(const math::VectorXd& x) const {

  // CROBOT_ASSERT_ERROR(false, "Not implemented");

  math::MatrixXd A            = get_dcdx(x);
  math::VectorXd constratints = c(x);

  math::VectorXd b = (A * x - constratints);

  return std::make_tuple(A, b);
}

//==============================================================================
math::VectorXd PoseTrackingSystem::get_c_eps() const {
  return m_c_eps;
}

//==============================================================================
kdl::DynamicGrasp::ConstPtr PoseTrackingSystem::get_grasp() const {
  return m_robot_state->get_grasp();
}
