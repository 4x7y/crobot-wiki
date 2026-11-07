#pragma once

#include "crobot/common/common.hpp"
#include "crobot/ekf/least_square_system.hpp"
#include "crobot/kdl/kdl.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ekf {

class CamTrackingSystem : public LeastSquareSystem {
public:
  using Ptr      = std::shared_ptr<CamTrackingSystem>;
  using ConstPtr = std::shared_ptr<const CamTrackingSystem>;
  /// Estimate camera world pose from the robot's rh_forearm marker site.
  /// States and observations are [x, y, z, qw, qx, qy, qz]; const_dim must
  /// be 1.
  CamTrackingSystem(
      const math::VectorXd& x0,
      const math::MatrixXd& P0,
      const math::MatrixXd& R,
      int const_dim,
      kdl::ConstRigidBodyTreePtr robot);

  /// Camera calibration from a marker with known world pose (wxyz quaternion).
  CamTrackingSystem(
      const math::VectorXd& x0,
      const math::MatrixXd& P0,
      const math::MatrixXd& R);
  /// Set the marker's known world transform before each observation.
  void set_marker_transform(const math::Matrix4d& transform);
  math::VectorXd innovation(
      const math::VectorXd& measured,
      const math::VectorXd& predicted) const override;

  math::VectorXd h(
      const math::VectorXd& x, const math::VectorXd& v) const override;

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

  void update_sensor_state(const math::VectorXd& qpos);

  std::tuple<math::MatrixXd, math::VectorXd> get_linear_constraints(
      const math::VectorXd& x) const override;

  math::VectorXd get_c_eps() const override;

private:
  kdl::ConstRigidBodyTreePtr m_robot;
  kdl::ConstRigidBodyPtr m_arm;
  kdl::ConstSitePtr m_site;
  math::Matrix4d m_marker_xform = math::Matrix4d::Identity();
};
} // namespace crobot::ekf
