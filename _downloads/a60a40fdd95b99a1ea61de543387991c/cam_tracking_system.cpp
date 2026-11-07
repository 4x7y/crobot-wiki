#include "crobot/ekf/cam_tracking_system.hpp"

#include <stdexcept>

using namespace crobot;
using namespace crobot::ekf;

namespace {
math::Quaternion rotation(const math::VectorXd& pose) {
  if (pose.size() != 7)
    throw std::invalid_argument("Camera pose must have seven coordinates");
  if (!pose.allFinite() || pose.tail<4>().norm() < 1e-12)
    throw std::runtime_error("Invalid camera pose quaternion");
  return math::Quaternion(pose(3), pose(4), pose(5), pose(6));
}
} // namespace

CamTrackingSystem::CamTrackingSystem(
    const math::VectorXd& x0,
    const math::MatrixXd& P0,
    const math::MatrixXd& R,
    int const_dim,
    kdl::ConstRigidBodyTreePtr robot)
  : CamTrackingSystem(x0, P0, R) {
  if (!robot || const_dim != 1)
    throw std::invalid_argument(
        "Camera model requires a robot and one constraint");
  m_robot = robot;
  m_arm   = m_robot->get_body("rh_forearm");
  if (!m_arm)
    throw std::invalid_argument("Camera marker body not found");
  m_site = m_arm->get_site("site:forearm_marker");
  if (!m_site)
    throw std::invalid_argument("Camera marker site not found");
  update_sensor_state(m_robot->make_zero_qpos());
}

CamTrackingSystem::CamTrackingSystem(
    const math::VectorXd& x0, const math::MatrixXd& P0, const math::MatrixXd& R)
  : LeastSquareSystem(x0, P0, R, 1) {
  if (x0.size() != 7 || R.rows() != 7 || x0.tail<4>().norm() < 1e-12)
    throw std::invalid_argument("Camera model requires a seven-element pose");
}

void CamTrackingSystem::set_marker_transform(const math::Matrix4d& transform) {
  const math::Matrix3d R = transform.block<3, 3>(0, 0);
  if (!transform.allFinite()
      || !(R.transpose() * R).isApprox(math::Matrix3d::Identity(), 1e-8)
      || std::abs(R.determinant() - 1) > 1e-8
      || !transform.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1)))
    throw std::invalid_argument("Invalid marker rigid transform");
  m_marker_xform = transform;
}

math::VectorXd CamTrackingSystem::innovation(
    const math::VectorXd& measured, const math::VectorXd& predicted) const {
  math::VectorXd aligned = measured;
  if (aligned.tail<4>().dot(predicted.tail<4>()) < 0)
    aligned.tail<4>() *= -1;
  return aligned - predicted;
}

math::VectorXd CamTrackingSystem::h(
    const math::VectorXd& x, const math::VectorXd& v) const {
  if (v.size() != 7)
    throw std::invalid_argument(
        "Camera observation noise must have seven coordinates");
  const auto camera = rotation(x).normalized();
  math::VectorXd y(7);
  y.head<3>()
      = camera.conjugate() * (m_marker_xform.tform2trvec() - x.head<3>());
  y.tail<4>() = (camera.conjugate() * m_marker_xform.tform2quat()).as_vector();
  return y + v;
}

math::SparseJacobian CamTrackingSystem::get_dhdx(
    const math::VectorXd& x, const math::VectorXd&) const {
  const auto camera   = rotation(x);
  const auto marker   = m_marker_xform.tform2quat();
  math::MatrixXd H    = math::MatrixXd::Zero(7, 7);
  H.block<3, 3>(0, 0) = -camera.normalized().toRotationMatrix().transpose();
  H.block<3, 4>(0, 3) = camera.nonunit_diff_conjugate_rotvec_coeffs(
      m_marker_xform.tform2trvec() - x.head<3>());
  math::Matrix4d derivative    = math::Matrix4d::Zero();
  derivative(0, 0)             = marker.w();
  derivative.block<1, 3>(0, 1) = marker.vec().transpose();
  derivative.block<3, 1>(1, 0) = marker.vec();
  derivative.block<3, 3>(1, 1)
      = -marker.w() * math::Matrix3d::Identity() + marker.vec().skew();
  H.block<4, 4>(3, 3) = derivative * camera.unit_deriv();
  return H.sparseView();
}

math::SparseJacobian CamTrackingSystem::get_dhdv(
    const math::VectorXd&, const math::VectorXd&) const {
  return math::MatrixXd::Identity(7, 7).sparseView();
}

math::VectorXd CamTrackingSystem::c(const math::VectorXd& x) const {
  return math::VectorXd::Constant(1, x.tail<4>().squaredNorm() - 1);
}

math::SparseJacobian CamTrackingSystem::get_dcdx(
    const math::VectorXd& x) const {
  math::SparseJacobian C(1, 7);
  for (int i = 0; i < 4; ++i)
    C.coeffRef(0, i + 3) = 2 * x(i + 3);
  return C;
}

std::tuple<
    std::vector<math::MatrixXd>,
    std::vector<math::VectorXd>,
    std::vector<double>>
CamTrackingSystem::get_constraints(const math::VectorXd&) const {
  math::MatrixXd A = math::MatrixXd::Zero(7, 7);
  A.bottomRightCorner<4, 4>().setIdentity();
  return {{A}, {math::VectorXd::Zero(7)}, {-1}};
}

void CamTrackingSystem::update_sensor_state(const math::VectorXd& qpos) {
  if (!m_robot || qpos.size() != m_robot->get_qpos_dim() || !qpos.allFinite())
    throw std::invalid_argument("Invalid camera robot sensor state");
  const auto tforms = m_robot->forward_kinematics(qpos);
  m_marker_xform    = m_arm->get_xform(tforms) * m_site->get_tform();
}

std::tuple<math::MatrixXd, math::VectorXd>
CamTrackingSystem::get_linear_constraints(const math::VectorXd& x) const {
  math::MatrixXd A = get_dcdx(x);
  return {A, A * x - c(x)};
}

math::VectorXd CamTrackingSystem::get_c_eps() const {
  return math::VectorXd::Constant(1, 5e-4);
}
