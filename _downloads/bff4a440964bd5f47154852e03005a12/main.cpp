#include <iostream>
#include <string>
#include <thread>

#include <time.h>

#include "crobot/common/common.hpp"
#include "crobot/control/control.hpp"
#include "crobot/ik/ik.hpp"
#include "crobot/planning/trapvel_time_scaling.hpp"
#include "crobot/simulation/simulation.hpp"

using namespace crobot;
using namespace math;
using namespace simulation;
using namespace control;
using namespace kdl;
using namespace planning;

enum ControlMode
{
  HYBRID_POS_IMPD_CTRL,
  HYBRID_POS_XFRC_CTRL,
  TASK_SPACE_XPOS_CTRL,
};

class MySim : public simulation::MJSim {
public:
  MySim(const std::string& filename, ControlMode mode) : MJSim(filename) {
    m_robot     = get_rbtree();
    m_ee        = m_robot->get_body("link6");
    m_simctc_ee = add_contact("ee", "floor");
    m_robot->show_details();

    m_bpos_ee        = math::Vector3d(-0.2, 0.0, 0.0);
    m_rotm_ee.col(0) = math::Vector3d::UnitZ();
    m_rotm_ee.col(1) = -math::Vector3d::UnitY();
    m_rotm_ee.col(2) = math::Vector3d::UnitX();

    m_xpos_ee_start        = math::Vector3d(-0.1, 0.4, 0);
    m_xmat_ee_start.col(0) = math::Vector3d::UnitX();
    m_xmat_ee_start.col(1) = math::Vector3d::UnitY();
    m_xmat_ee_start.col(2) = math::Vector3d::UnitZ();

    m_xpos_ee_end        = math::Vector3d(0.1, 0.4, 0);
    m_xmat_ee_end.col(0) = math::Vector3d::UnitX();
    m_xmat_ee_end.col(1) = math::Vector3d::UnitY();
    m_xmat_ee_end.col(2) = math::Vector3d::UnitZ();

    math::Matrix4d xform_ee_start    = math::Matrix4d::Identity();
    xform_ee_start.block(0, 0, 3, 3) = m_xmat_ee_start;
    xform_ee_start.block(0, 3, 3, 1) = m_xpos_ee_start;

    math::Matrix4d xform_ee_end    = math::Matrix4d::Identity();
    xform_ee_end.block(0, 0, 3, 3) = m_xmat_ee_end;
    xform_ee_end.block(0, 3, 3, 1) = m_xpos_ee_end;

    TrapvelTimeScaling::ConstPtr time_scaling
        = std::make_shared<TrapvelTimeScaling>(100.0);

    m_traj = std::make_shared<PoseTrajectory>(
        xform_ee_start, xform_ee_end, time_scaling);

    m_desired_frame = WorldFrame::MakeShared(xform_ee_start);

    m_ctrl = TaskSpaceController::MakeShared(m_robot);
    if (mode == HYBRID_POS_IMPD_CTRL) {
      TaskImpdCtrlTargetPtr target = m_ctrl->add_hyrbid_impd_target(
          m_desired_frame,
          m_ee,
          m_rotm_ee,
          m_bpos_ee,
          {0, 1, 2, 3, 4},
          {5},
          m_simctc_ee);
      target->set_desired_force(math::Vector1d(-3.0));
    }
    else if (mode == HYBRID_POS_XFRC_CTRL) {
      TaskXfrcCtrlTargetPtr target = m_ctrl->add_hyrbid_force_target(
          m_desired_frame,
          m_ee,
          m_rotm_ee,
          m_bpos_ee,
          {0, 1, 2, 3, 4},
          {5},
          m_simctc_ee);
      target->set_desired_force(math::Vector1d(-3.0));
    }
    else if (mode == TASK_SPACE_XPOS_CTRL) {
      m_xpos_ee_start = math::Vector3d(0.0, 0.5, 0.3);
      m_ctrl->add_xpos_target(
          m_desired_frame, m_ee, m_rotm_ee, m_bpos_ee, m_simctc_ee);
    }

    // set up the IK problem
    ik::GeneralizedIK ik(m_robot);
    ik.add_position_target(
        "xpos_link6_ee", "link6", m_bpos_ee, m_xpos_ee_start);
    ik.add_cartesian_target(
        "xpos_link3",
        "link3",
        math::Vector3d(-10, -10, 0.2),
        math::Vector3d(10, 10, 10));
    ik.add_orientation_target(
        "xmat_link6",
        "link6",
        math::Quaternion(m_xmat_ee_start * m_rotm_ee.transpose()));
    ik.set_init_qpos(
        (math::VectorXd(6) << -3.12, -0.429, -4.5, -2.51, 4.71, -1.55)
            .finished());
    ik.solve();
    // set the result
    m_robot->set_qpos_mj(ik.get_qpos());
    m_robot->forward_dynamics_mj();
  }

  void visualize(simulation::MarkerManager& markers) override {
    markers.add_frame_marker(
        m_ee->get_xpos(m_bpos_ee), m_ee->get_xmat() * m_rotm_ee);

    // draw contact force
    if (m_simctc_ee->is_active()) {
      markers.add_arrow_marker(
          m_simctc_ee->get_xpos(),
          m_simctc_ee->get_xpos() + m_simctc_ee->get_xfrc() * 0.05,
          0.002,
          Vector4d(1, 0, 0, 1));

      markers.add_arrow_marker(
          m_simctc_ee->get_xpos(),
          m_simctc_ee->get_xpos()
              + m_xmat_ee_start.transpose() * math::Vector3d(0, 0, -3.0) * 0.05,
          0.002,
          Vector4d(0, 1, 1, 1));
    }
  }

  void control() override {
    // update desired frame
    math::Matrix4d xform_desired = m_traj->get_tform(m_time);
    math::VectorXd xvel_desired  = m_traj->get_velocity(m_time);
    math::VectorXd xacc_desired  = m_traj->get_acceleration(m_time);
    m_desired_frame->set_xform(xform_desired);
    m_desired_frame->set_xvel(xvel_desired);
    m_desired_frame->set_xacc(xacc_desired);

    m_time = m_time + m_dt;

    math::VectorXd qpos          = m_robot->get_qpos_mj();
    math::VectorXd qvel          = m_robot->get_qvel_mj();
    math::VectorXd ctrl_resolved = m_ctrl->compute(qpos, qvel);
    m_robot->set_ctrl_mj(ctrl_resolved);
  }

private:
  ConstMJRBTreePtr m_robot;
  ConstMJRigidBodyPtr m_ee;
  TaskSpaceControllerPtr m_ctrl;
  simulation::ConstContactPtr m_simctc_ee;

  math::Vector3d m_bpos_ee;
  math::Matrix3d m_rotm_ee;

  math::Vector3d m_xpos_ee_start;
  math::Matrix3d m_xmat_ee_start;
  math::Vector3d m_xpos_ee_end;
  math::Matrix3d m_xmat_ee_end;

  WorldFramePtr m_desired_frame;

  PoseTrajectory::Ptr m_traj;

  double m_time     = 0.0;
  const double m_dt = 0.02;
};

int main(int argc, char const* argv[]) {
  common::ArgumentParser parser(
      "This is a program showing the usage of hybrid controller.");
  common::HelpFlag help(
      parser, "help", "Display this help menu.", {'h', "help"});
  common::ValueFlag<std::string> control_law(
      parser,
      "control",
      "Control law to be demonstrated. Options: force, position, impedance.",
      {'c'},
      "force");
  try {
    parser.ParseCLI(argc, argv);
  }
  catch (const common::Help&) {
    std::cout << parser;
    return 0;
  }
  catch (const common::ParseError& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }
  catch (const common::ValidationError& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  auto control_mode = [&]() {
    if (control_law.Get() == "force") {
      return HYBRID_POS_XFRC_CTRL;
    }
    else if (control_law.Get() == "impedance") {
      return HYBRID_POS_IMPD_CTRL;
    }
    else if (control_law.Get() == "position") {
      return TASK_SPACE_XPOS_CTRL;
    }
    else {
      CROBOT_LOG_ERROR("Invalid control law: {}", control_law.Get());
      exit(1);
    }
  }();

  MySim sim("../resources/ur5/ur5.xml", control_mode);
  sim.run();

  return 0;
}
