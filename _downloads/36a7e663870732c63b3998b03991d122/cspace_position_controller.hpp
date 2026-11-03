#pragma once

#include <variant>

#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

// forward declaration
class CSpacePositionController;
using CSpacePositionControllerPtr = std::shared_ptr<CSpacePositionController>;
using ConstCSpacePositionControllerPtr
    = std::shared_ptr<const CSpacePositionController>;

class CSpacePositionController {
public:
  static CSpacePositionControllerPtr MakeShared(
      kdl::ConstRigidBodyTreePtr robot,
      math::VectorXd active_id,
      const std::string& params_file);

  CSpacePositionController(
      kdl::ConstRigidBodyTreePtr robot,
      math::VectorXd active_id,
      const std::string& params_file);

  void reset();

  math::VectorXd compute(
      const math::VectorXd& qpos_all,
      const math::VectorXd& qvel_all,
      double time);

  void set_qpos_desired(const math::VectorXd& qpos);

  void set_qvel_desired(const math::VectorXd& qvel);

  void set_qacc_desired(const math::VectorXd& qacc);

private:
  kdl::ConstRigidBodyTreePtr m_robot;
  math::VectorXd m_qpos_desired;
  math::VectorXd m_qvel_desired;
  math::VectorXd m_qacc_desired;
  math::VectorXd m_qpos_error_int;

  double m_time_last_update;
  int m_nq;
  int m_nq_active;
  math::VectorXd m_active_id;

  math::MatrixXd m_kp;
  math::MatrixXd m_ki;
  math::MatrixXd m_kd;
};

} // namespace crobot::control

