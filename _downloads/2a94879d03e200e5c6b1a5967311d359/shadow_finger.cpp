#include "crobot/hardware/shadow_finger.hpp"

#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/optimization/cons_friction_cone.hpp"
#include "crobot/optimization/constraint_set.hpp"
#include "crobot/optimization/variable_set.hpp"

using namespace crobot;
using namespace hardware;
using namespace optimization;

//==============================================================================
class ForceVariable : public VariableSet {
public:
  ForceVariable() : VariableSet(3, "force") {
  }

  void set_variables(const math::VectorXd& value) override {
    m_value = value;
  }

  math::VectorXd get_values() const override {
    return m_value;
  }

  VecBound get_bounds() const override {
    VecBound bounds(3);
    bounds.at(0) = Bounds(-5, 5);
    bounds.at(1) = Bounds(-5, 5);
    bounds.at(2) = Bounds(-5, 5);
    return bounds;
  }

private:
  math::Vector3d m_value;
};

//==============================================================================
struct TorqueCost : public optimization::CostTerm
{
  TorqueCost() : CostTerm("torque_cost", {"force"}) {
  }

  double get_cost(const math::VectorXd& x) const override {
    return (jacp.transpose() * x - qfrc).squaredNorm();
  }

  void fill_jacobian_block(
      Component::Ptr var_set, math::SparseJacobian& jac) const override {
    if (var_set->get_name() == "force") {
      math::Vector3d f    = var_set->get_values();
      math::Matrix3d Q    = jacp * jacp.transpose();
      math::Vector3d cjac = 2.0 * Q * f - 2.0 * jacp * qfrc;

      jac.coeffRef(0, 0) = cjac(0);
      jac.coeffRef(0, 1) = cjac(1);
      jac.coeffRef(0, 2) = cjac(2);
    }
  }

  math::VectorXd qfrc;
  math::LinearJacobian jacp;
};

//==============================================================================
ShadowFinger::ShadowFinger(
    kdl::ConstRigidBodyTreePtr hand,
    const std::string& name,
    int id,
    const std::string& tip_bodyname,
    const std::string& tip_sitename)
  : m_hand(hand),
    m_name(name),
    m_id(id),
    m_tip_bodyname(tip_bodyname),
    m_tip_sitename(tip_sitename) {
  kdl::ConstSitePtr site_btsp
      = m_hand->get_body(m_tip_bodyname)->get_site(m_tip_sitename);
  math::Quaternion quat_site = site_btsp->get_quat();
  double radius_b            = site_btsp->get_size()(1);

  math::Vector3d btsp_center(0, -radius_b, 0);
  m_bpos_btsp_ctc = site_btsp->get_bpos() + quat_site * btsp_center;

  m_problem = std::make_shared<optimization::Problem>();
  m_problem->add_variable_set(std::make_shared<ForceVariable>());
  m_problem->add_cost_set(std::make_shared<TorqueCost>());
  m_problem->add_constraint_set(std::make_shared<FrictionConeConstraint>(
      "friction_cone", "force", 0.5, math::Vector3d::UnitZ()));

  m_solver = std::make_shared<optimization::IpoptSolver>();
  m_solver->set_option("jacobian_approximation", "exact");
  m_solver->set_option("linear_solver", "mumps");
  m_solver->set_option("print_level", 0);
}

//==============================================================================
const std::string& ShadowFinger::get_name() const {
  return m_name;
}

//==============================================================================
std::vector<int> ShadowFinger::get_motor_ids() const {
  std::vector<int> motor_ids;
  for (const ShadowMotor::Ptr& motor : m_motors) {
    motor_ids.push_back(motor->get_id());
  }
  return motor_ids;
}

//==============================================================================
std::vector<int> ShadowFinger::get_qpos_addrs() const {
  std::vector<int> qpos_addrs;
  for (const ShadowJoint::Ptr& joint : m_joints) {
    qpos_addrs.push_back(joint->get_qpos_addr());
  }
  return qpos_addrs;
}

//==============================================================================
std::vector<std::string> ShadowFinger::get_motor_names() const {
  std::vector<std::string> motor_names;
  for (const ShadowMotor::Ptr& motor : m_motors) {
    motor_names.push_back(motor->get_name());
  }
  return motor_names;
}

//==============================================================================
std::vector<std::string> ShadowFinger::get_joint_names() const {
  std::vector<std::string> joint_names;
  for (const ShadowJoint::Ptr& joint : m_joints) {
    joint_names.push_back(joint->get_name());
  }
  return joint_names;
}

//==============================================================================
math::VectorXd ShadowFinger::convert_pwm2nm(
    const math::VectorXd& pwm_active,
    const math::VectorXd& qpos_hand,
    const math::VectorXd& qfrc_guess_active) const {
  math::VectorXd qfrc_active(m_motors_active.size());
  for (std::size_t i = 0; i < m_motors_active.size(); i++) {
    qfrc_active(i) = m_motors_active[i]->get_torque_pwm2nm(
        pwm_active[i], qfrc_guess_active[i], qpos_hand);
  }
  return qfrc_active;
}

//==============================================================================
std::pair<math::VectorXd, math::VectorXd> ShadowFinger::get_sg_pwm_finger(
    const math::VectorXd& sg_pwm_hand) const {
  CROBOT_ASSERT_ERROR(sg_pwm_hand.size() == 40);
  math::VectorXd sg_pos(m_motors_active.size());
  math::VectorXd sg_neg(m_motors_active.size());
  for (std::size_t i = 0; i < m_motors_active.size(); i++) {
    int index_pos = m_motors_active[i]->m_strain_gauge.index_pos_torque;
    int index_neg = m_motors_active[i]->m_strain_gauge.index_neg_torque;
    sg_pos[i]     = sg_pwm_hand[index_pos];
    sg_neg[i]     = sg_pwm_hand[index_neg];
  }
  return std::make_pair(sg_pos, sg_neg);
}

//==============================================================================
math::VectorXd ShadowFinger::get_sg_pwm_active(
    const math::VectorXd& sg_pwm_hand, const math::VectorXd& qfrc_est) const {
  math::VectorXd sg_pwm_pos;
  math::VectorXd sg_pwm_neg;
  std::tie(sg_pwm_pos, sg_pwm_neg) = get_sg_pwm_finger(sg_pwm_hand);

  math::VectorXd strain_gauge_active(m_motors_active.size());
  for (size_t i = 0; i < m_motors_active.size(); i++) {
    strain_gauge_active[i] = qfrc_est(i) > 0 ? sg_pwm_neg[i] : sg_pwm_pos[i];
  }
  return strain_gauge_active;
}

//==============================================================================
math::VectorXd ShadowFinger::get_qpos_active(const math::VectorXd& qpos) const {
  return qpos(Eigen::all, get_qpos_addrs_active(qpos));
}

//==============================================================================
std::vector<int> ShadowFinger::get_qpos_addrs_active(
    const math::VectorXd& qpos) const {
  std::vector<int> qpos_addrs_active;
  for (const ShadowMotor::Ptr& motor : m_motors_active) {
    qpos_addrs_active.push_back(motor->get_active_joint_qpos_addr(qpos));
  }
  return qpos_addrs_active;
}

//==============================================================================
math::VectorXd ShadowFinger::get_qfrc_guess_active(
    const math::VectorXd& qfrc_guess_hand, const math::VectorXd& qpos) const {
  return qfrc_guess_hand(Eigen::all, get_qpos_addrs_active(qpos));
}

//==============================================================================
math::Vector3d ShadowFinger::estimate_contact_force(
    const math::Vector3d& bpos,         // contact position
    const math::Vector3d& xvec,         // contact normal outward pointing
    const math::VectorXd& qpos_hand,    // hand joint positions
    const math::VectorXd& sg_pwm_hand,  // strain gauge pwm readings
    ShadowFinger::EstimateAlgorithm alg //
) const {
  CROBOT_ASSERT_ERROR(
      std::abs(xvec.norm() - 1.0) < 1e-3, "xvec is not a unit vector");
  CROBOT_ASSERT_ERROR(qpos_hand.size() == 24);
  CROBOT_ASSERT_ERROR(sg_pwm_hand.size() == 40);

  std::vector<math::Matrix4d> tforms = m_hand->forward_kinematics(qpos_hand);
  math::Jacobian jacb = m_hand->get_body(m_tip_bodyname)->get_jacb(tforms);
  math::LinearJacobian jacp
      = m_hand->get_body(m_tip_bodyname)->get_jacp_at_bpos(tforms, jacb, bpos);
  math::LinearJacobian jacp_btsp_active
      = jacp(Eigen::all, get_qpos_addrs_active(qpos_hand));

  // determines which strain gauge is active to compute the force
  math::VectorXd qfrc_guess_active = jacp_btsp_active.transpose() * xvec;

  std::cout << "qfrc_guess_active: " << qfrc_guess_active.transpose()
            << std::endl;

  math::VectorXd sg_pwm_active
      = get_sg_pwm_active(sg_pwm_hand, qfrc_guess_active);
  math::VectorXd qfrc_active
      = convert_pwm2nm(sg_pwm_active, qpos_hand, qfrc_guess_active);

  std::cout << "sg_pwm_active: " << sg_pwm_active.transpose() << std::endl;
  std::cout << "qfrc_active: " << qfrc_active.transpose() << std::endl;

  // Eigen::BDCSVD<math::Matrix3X> svd(jacp_btsp_active);
  // CROBOT_LOG_INFO("sg_pwm_active: {}", sg_pwm_active.transpose());
  // CROBOT_LOG_INFO("qfrc_active: {}", qfrc_active.transpose());
  // CROBOT_LOG_INFO("jacp singular values: {}",
  // svd.singularValues().transpose());

  math::Vector3d xfrc_est;
  bool is_opt_solved = false;
  if (alg == EstimateAlgorithm::FRICTION_CONE_OPTIMIZATION) {
    // update cost
    std::shared_ptr<TorqueCost> optcost_torque_diff
        = std::dynamic_pointer_cast<TorqueCost>(
            m_problem->get_costs().get_component("torque_cost"));
    optcost_torque_diff->jacp = jacp_btsp_active;
    optcost_torque_diff->qfrc = qfrc_active;

    // update friction cone constraint
    std::shared_ptr<FrictionConeConstraint> optcons_friction_cone
        = std::dynamic_pointer_cast<FrictionConeConstraint>(
            m_problem->get_constraints().get_component("friction_cone"));
    optcons_friction_cone->set_axis(xvec);

    // update variable initial guess
    std::shared_ptr<ForceVariable> optvars_force
        = std::dynamic_pointer_cast<ForceVariable>(
            m_problem->get_variables()->get_component("force"));
    optvars_force->set_variables(xvec);

    // solve for optimization problem
    m_solver->solve(*m_problem);

    if (m_solver->get_return_status() == IpoptSolver::SUCCEEDED) {
      is_opt_solved = true;
      xfrc_est      = m_problem->get_variable_values();
    }
    else {
      CROBOT_LOG_WARN(
          "Contact force cannot be estimated via friction cone optimization. "
          "Switching to direct method.");
      CROBOT_LOG_WARN("xvec             = {}", xvec.transpose());
      CROBOT_LOG_WARN("qfrc_active      = {}", qfrc_active.transpose());
      CROBOT_LOG_WARN("jacp_btsp_active = \n{}", jacp_btsp_active);
    }
  }

  if (alg == EstimateAlgorithm::DIRECT || !is_opt_solved) {
    xfrc_est
        //     = math::calculate_jacb_inv(jacp_btsp_active.transpose()) *
        //     qfrc_active;
        = jacp_btsp_active.transpose().colPivHouseholderQr().solve(qfrc_active);

    // map xfrc_est to xvec
    // xfrc_est = xfrc_est.transpose() * xvec * xvec;

    std::cout << "jacp_btsp_active: "
              << math::calculate_jacb_inv(jacp_btsp_active.transpose())
              << std::endl;
  }
  else if (alg != EstimateAlgorithm::FRICTION_CONE_OPTIMIZATION) {
    CROBOT_ASSERT_ERROR(false, "Unknown estimate algorithm");
  }

  // solve for f in J^T f = qfrc_active
  // least squares solution to get the force applied to the finger
  return xfrc_est;
}
