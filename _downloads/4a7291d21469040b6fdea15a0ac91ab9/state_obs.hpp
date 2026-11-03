#pragma once

#include "crobot/common/common.hpp"
#include "crobot/kdl/kdl.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ekf {

class StateObs {
public:
  using Ptr      = std::shared_ptr<StateObs>;
  using ConstPtr = std::shared_ptr<const StateObs>;

  StateObs(kdl::DynamicGrasp::ConstPtr grasp, kdl::ConstRigidBodyTreePtr robot);

  void update_sensor_state(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      std::vector<math::Vector3d> forces);

  void update_obj_state(
      const math::VectorXd& obj_pos,
      const math::Vector6d& obj_bvel,
      std::vector<math::Vector2d> obj_surf_params,
      std::vector<math::Vector2d> fin_surf_params);

  // get the contact velocities (in surface parametrization) from grasp
  // return (sigma_dot_i, eta_dot_i)
  std::vector<std::pair<math::Vector2d, math::Vector2d>> get_ctc_vels() const;

  // get contact forces from the grasp
  std::vector<math::Vector3d> get_xfrc() const;

  // get d(surface param vels) 4n DOF
  // d(obj quaternion) 4n x 4
  // d(obj angular velocity) 4n x 3
  // d(ctc position) 4n x 4n
  std::tuple<math::MatrixXd, math::MatrixXd, math::MatrixXd> get_dsv() const;

  // get d(contact constraints) 6 DOF
  // d(obj pos) 6n x 3
  // d(obj quat) 6n x 4
  // d(surf_params) 6n x 4n
  std::tuple<math::MatrixXd, math::MatrixXd, math::MatrixXd> get_dctc_cons()
      const;

  kdl::DynamicGrasp::ConstPtr get_grasp() const;

  std::vector<math::Matrix4d> get_tforms() const;

private:
  std::vector<math::Matrix4d> m_tforms;
  math::VectorXd m_qpos;
  math::VectorXd m_qvel;

  math::VectorXd m_obj_pos;
  math::Vector6d m_obj_bvel;

  kdl::DynamicGrasp::ConstPtr m_grasp;
  kdl::ConstRigidBodyTreePtr m_robot;
  kdl::ConstRigidBodyPtr m_obj;

  std::vector<math::Vector3d> m_xfrc;

  std::vector<int> m_obj_qpos_addr;
  std::vector<int> m_obj_qvel_addr;

  std::vector<math::Matrix32> m_A; // d(surf_obj)/d(uv_obj)
  std::vector<math::Matrix32> m_B; // d(surf_fin)/d(uv_fin)
  std::vector<math::Matrix32> m_C; // d(norm_obj)/d(uv_obj)
  std::vector<math::Matrix32> m_D; // d(norm_fin)/d(uv_fin)

  std::vector<std::pair<math::Vector2d, math::Vector2d>> m_ctc_vels;
};
} // namespace crobot::ekf

