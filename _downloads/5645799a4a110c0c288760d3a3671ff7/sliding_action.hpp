// #ifndef CROBOT_CONTROL_SLIDING_ACTION_HPP
// #define CROBOT_CONTROL_SLIDING_ACTION_HPP

// #include "crobot/control/action.hpp"
// #include "crobot/control/admittance_controller.hpp"
// #include "crobot/control/body_target.hpp"
// #include "crobot/kdl/dynamic_grasp.hpp"
// #include "crobot/planning/linear_trajectory.hpp"

// namespace crobot::control {

// class SlidingAction : public Action {
// public:
//   using Ptr      = std::shared_ptr<SlidingAction>;
//   using ConstPtr = std::shared_ptr<const SlidingAction>;

//   struct CtrlTarget
//   {
//     std::string body_name;
//     std::vector<int> dofs_controlled;
//     math::Vector3d bfrc_desired;
//   };

//   SlidingAction(
//       kdl::ConstRigidBodyTreePtr rbtree,
//       kdl::DynamicGrasp::ConstPtr grasp_prev,
//       kdl::DynamicGrasp::ConstPtr grasp_next,
//       const std::vector<CtrlTarget>& moving_targets,
//       const std::vector<CtrlTarget>& force_targets,
//       const math::Vector6d& xwrench_ext_desired,
//       double duration);

//   std::pair<math::VectorXd, math::VectorXd> get_command(
//       double time,
//       const math::VectorXd& qpos_actual,
//       const math::VectorXd& qpos_cmd,
//       const math::VectorXd& qvel_actual) override;

//   void initialize(
//       const math::VectorXd& qpos_actual,
//       const math::VectorXd& qpos_cmd) override;

//   void fix_joint(const std::string& joint_name) override;

//   void fix_joint(const std::string& joint_name, double value) override;

//   void fix_body(const std::string& body_name) override;

//   kdl::Grasp::ConstPtr get_grasp() const override;

// private:
//   void set_control_targets(
//       CtrlTarget& target, std::vector<math::Matrix4d>& tforms);

//   kdl::ConstRigidBodyTreePtr m_rbtree;
//   kdl::ConstRigidBodyPtr m_body;

//   // the one that is updated during the control
//   kdl::DynamicGrasp::ConstPtr m_grasp;
//   // our target grasp
//   kdl::DynamicGrasp::ConstPtr m_grasp_target;

//   std::vector<CtrlTarget> m_moving_targets;
//   std::vector<CtrlTarget> m_force_targets;

//   math::Vector6d m_xwrench_ext_desired;

//   control::AdmittanceController::Ptr m_controller;
//   std::vector<
//       std::pair<control::AdmittanceControlTarget::Ptr, kdl::WorldFramePtr>>
//       m_ctrl_targets;

//   std::vector<std::pair<
//       planning::LinearTrajectory::ConstPtr,
//       planning::LinearTrajectory::ConstPtr>>
//       m_bpos_targets;
//   math::VectorXd m_cfrc_internal_stack;

//   // body name to the contact index in grasp
//   std::unordered_map<std::string, std::size_t> m_name_to_index;
// };

// } // namespace crobot::control

// #endif // CROBOT_CONTROL_SLIDING_ACTION_HPP
