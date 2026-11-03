#pragma once

#include "crobot/common/common.hpp"
#include "crobot/ekf/ekf.hpp"
#include "crobot/ekf/obj_dynamics.hpp"
#include "crobot/ekf/state_obs.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ekf {

class PoseTrackingSystem : public ekf::KalmanFilterSystem {
public:
  using Ptr      = std::shared_ptr<PoseTrackingSystem>;
  using ConstPtr = std::shared_ptr<const PoseTrackingSystem>;
  /**
   * @brief Construct a new Kalman Filter System object
   *
   * @param x0 The initial state vector
   * @param u0 The input vector
   * @param P0 The initial state covariance matrix
   * @param Q The process w noise covariance matrix
   * @param R The measurement v noise covariance matrix
   */
  PoseTrackingSystem(
      const math::VectorXd& x0,
      const math::VectorXd& u0,
      const math::MatrixXd& P0,
      const math::MatrixXd& Q,
      const math::MatrixXd& R,
      const math::MatrixXd& P0_reset,
      int const_dim,
      const std::string& c_eps_file,
      ObjDynamics::Ptr obj_dynamics,
      StateObs::Ptr robot_state);

  PoseTrackingSystem(
      const math::VectorXd& x0,
      const math::VectorXd& u0,
      const math::MatrixXd& P0,
      const math::MatrixXd& Q,
      const math::MatrixXd& R,
      const math::MatrixXd& P0_reset,
      int const_dim,
      YAML::Node c_eps_node,
      ObjDynamics::Ptr obj_dynamics,
      StateObs::Ptr robot_state);

  math::VectorXd f(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w) const override;

  math::VectorXd h(
      const math::VectorXd& x, const math::VectorXd& v) const override;

  math::SparseJacobian get_dfdx(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w) const override;

  math::SparseJacobian get_dfdw(
      const math::VectorXd& x,
      const math::VectorXd& u,
      const math::VectorXd& w) const override;

  math::SparseJacobian get_dhdx(
      const math::VectorXd& x, const math::VectorXd& v) const override;

  math::SparseJacobian get_dhdv(
      const math::VectorXd& x, const math::VectorXd& v) const override;

  std::tuple<
      std::vector<math::MatrixXd>,
      std::vector<math::VectorXd>,
      std::vector<double>>
  get_constraints(const math::VectorXd& x) const override;

  math::VectorXd c(const math::VectorXd& x) const override;

  math::SparseJacobian get_dcdx(const math::VectorXd& x) const override;

  std::tuple<
      math::VectorXd,
      math::Vector6d,
      std::vector<math::Vector2d>,
      std::vector<math::Vector2d>,
      math::VectorXd>
  unpack_x(const math::VectorXd& x) const;

  void update_sensor_state(
      const math::VectorXd& qpos,
      const math::VectorXd& qvel,
      std::vector<math::Vector3d> forces);

  void update_obj_state(const math::VectorXd& x);

  std::tuple<math::MatrixXd, math::VectorXd> get_linear_constraints(
      const math::VectorXd& x) const override;

  math::VectorXd get_c_eps() const override;

  kdl::DynamicGrasp::ConstPtr get_grasp() const;

private:
  ObjDynamics::Ptr m_obj_dynamics;
  StateObs::Ptr m_robot_state;

  math::VectorXd m_c_eps;
};
} // namespace crobot::ekf

