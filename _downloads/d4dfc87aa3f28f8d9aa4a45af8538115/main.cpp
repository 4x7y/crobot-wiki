#include <iostream>
#include <string>
#include <thread>

#include "crobot/common/common.hpp"
#include "crobot/control/control.hpp"
#include "crobot/ik/ik.hpp"
#include "crobot/simulation/simulation.hpp"

using namespace crobot;
using namespace math;
using namespace kdl;
using namespace control;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

class MySim : public simulation::MJSim {
public:
  //============================================================================
  MySim(const std::string& filename, bool is_visualized)
    : MJSim(filename, is_visualized) {
    m_simctc_th = add_contact("rh_thtip", "box");
    m_simctc_ff = add_contact("rh_fftip", "box");

    m_robot = get_rbtree("wam/shoulder_yaw_link");
    m_arm   = SubChainController::MakeShared(
        RBSubChain::MakeShared(m_robot, "wam/shoulder_yaw_link", "rh_palm"),
        m_robot->get_body("rh_palm"),
        math::EulerAngleSeq::ZYX,
        {0, 1, 2});
    m_rh_th = SubChainController::MakeShared(
        RBSubTree::MakeShared(m_robot, "rh_thbase"),
        m_robot->get_body("rh_thdistal"),
        math::EulerAngleSeq::ZYX,
        {0, 1, 3, 4, 5},
        {2},
        m_simctc_th);
    m_rh_th->sfrc_desired = math::Vector1d(0.2);
    m_rh_ff               = SubChainController::MakeShared(
        RBSubTree::MakeShared(m_robot, "rh_ffknuckle"),
        m_robot->get_body("rh_ffdistal"),
        math::EulerAngleSeq::ZYX,
        {0, 1, 4, 5},
        {2},
        m_simctc_ff);
    m_rh_ff->sfrc_desired = math::Vector1d(0.2);
    m_rh_mf               = SubChainController::MakeShared(
        RBSubTree::MakeShared(m_robot, "rh_mfknuckle"),
        m_robot->get_body("rh_mfdistal"),
        math::EulerAngleSeq::ZYX,
        {0, 1, 2});
    m_rh_rf = SubChainController::MakeShared(
        RBSubTree::MakeShared(m_robot, "rh_rfknuckle"),
        m_robot->get_body("rh_rfdistal"),
        math::EulerAngleSeq::ZYX,
        {0, 1, 2});
    m_rh_lf = SubChainController::MakeShared(
        RBSubTree::MakeShared(m_robot, "rh_lfmetacarpal"),
        m_robot->get_body("rh_lfdistal"),
        math::EulerAngleSeq::ZYX,
        {0, 1, 2, 3});
    m_controller = TaskSpaceController::MakeShared(m_robot);
    m_controller->add_subchain_controller(m_arm);
    m_controller->add_subchain_controller(m_rh_th);
    m_controller->add_subchain_controller(m_rh_ff);
    m_controller->add_subchain_controller(m_rh_mf);
    m_controller->add_subchain_controller(m_rh_rf);
    m_controller->add_subchain_controller(m_rh_lf);

    math::Matrix3d xmat_box = math::Quaternion(0.8776, 0, 0, 0.4794).matrix();
    math::Vector3d xpos_box = math::Vector3d(0.8, 0.0, 0.7);

    m_rh_th->xpos_desired
        = xpos_box + xmat_box * math::Vector3d(0, +0.02, 0.08);
    m_rh_ff->xpos_desired
        = xpos_box + xmat_box * math::Vector3d(0, -0.02, 0.08);
    math::Vector3d xpos_th_btsp
        = xpos_box + xmat_box * math::Vector3d(0, +0.03, 0.08);
    math::Vector3d xpos_ff_btsp
        = xpos_box + xmat_box * math::Vector3d(0, -0.03, 0.08);

    // set up the IK problem
    ik::GeneralizedIK ik(m_robot);
    math::Vector3d bpos_btsp_center = math::Vector3d(0, -0.0018, 0.0205);
    math::AngleAxisd aaxis_btsp(-0.35, math::Vector3d::UnitX());
    math::Matrix3d rotm_btsp = aaxis_btsp.matrix();
    math::Vector3d bvec_tip  = -rotm_btsp.col(1); // -Y axis
    ik.add_position_target(
        "xpos_thdistal",
        m_rh_th->ee->get_name(),
        bpos_btsp_center,
        xpos_th_btsp);
    ik.add_aiming_target(
        "axang_thdistal",
        m_rh_th->ee->get_name(),
        bvec_tip,
        m_rh_th->xpos_desired,
        math::constantsd::pi() / 4.0);
    ik.add_position_target(
        "xpos_ffdistal",
        m_rh_ff->ee->get_name(),
        bpos_btsp_center,
        xpos_ff_btsp);
    ik.add_aiming_target(
        "axang_ffdistal",
        m_rh_ff->ee->get_name(),
        bvec_tip,
        m_rh_ff->xpos_desired,
        math::constantsd::pi() / 4.0);
    // solve for IK
    ik.solve();
    // set the result
    m_robot->set_qpos(data, ik.get_qpos());
    m_rh_mf->chain->set_qpos(data, Vector4d::Zero());
    m_rh_rf->chain->set_qpos(data, Vector4d::Zero());
    m_rh_lf->chain->set_qpos(data, Vector5d::Zero());
    m_robot->forward_all(data);

    m_arm->xpos_desired = m_arm->ee->get_xpos(data);
    m_arm->xmat_desired = m_arm->ee->get_xmat(data);
    m_arm->bpos         = math::Vector3d::Zero();
    m_arm->rotm         = math::Matrix3d::Identity();

    math::Matrix3d rotm_thctc2box;
    rotm_thctc2box.col(0)  = math::Vector3d::UnitX();
    rotm_thctc2box.col(1)  = math::Vector3d::UnitZ();
    rotm_thctc2box.col(2)  = -math::Vector3d::UnitY();
    m_rh_th->xmat_desired  = xmat_box * rotm_thctc2box;
    math::Matrix3d xmat_th = m_rh_th->ee->get_xmat(data);
    math::Vector3d xpos_th = m_rh_th->ee->get_xpos(data);
    m_rh_th->rotm          = xmat_th.transpose() * m_rh_th->xmat_desired;
    m_rh_th->bpos = xmat_th.transpose() * (m_rh_th->xpos_desired - xpos_th);

    math::Matrix3d rotm_ffctc2box;
    rotm_ffctc2box.col(0)  = math::Vector3d::UnitZ();
    rotm_ffctc2box.col(1)  = math::Vector3d::UnitX();
    rotm_ffctc2box.col(2)  = math::Vector3d::UnitY();
    m_rh_ff->xmat_desired  = xmat_box * rotm_ffctc2box;
    math::Matrix3d xmat_ff = m_rh_ff->ee->get_xmat(data);
    math::Vector3d xpos_ff = m_rh_ff->ee->get_xpos(data);
    m_rh_ff->rotm          = xmat_ff.transpose() * m_rh_ff->xmat_desired;
    m_rh_ff->bpos = xmat_ff.transpose() * (m_rh_ff->xpos_desired - xpos_ff);

    m_rh_mf->bpos         = bpos_btsp_center;
    m_rh_mf->rotm         = rotm_btsp;
    m_rh_mf->xmat_desired = m_rh_mf->ee->get_xmat(data, m_rh_mf->rotm);
    m_rh_mf->xpos_desired = m_rh_mf->ee->get_xpos(data, m_rh_mf->bpos);

    m_rh_rf->bpos         = bpos_btsp_center;
    m_rh_rf->rotm         = rotm_btsp;
    m_rh_rf->xmat_desired = m_rh_rf->ee->get_xmat(data, m_rh_rf->rotm);
    m_rh_rf->xpos_desired = m_rh_rf->ee->get_xpos(data, m_rh_rf->bpos);

    m_rh_lf->bpos         = bpos_btsp_center;
    m_rh_lf->rotm         = rotm_btsp;
    m_rh_lf->xmat_desired = m_rh_lf->ee->get_xmat(data, m_rh_lf->rotm);
    m_rh_lf->xpos_desired = m_rh_lf->ee->get_xpos(data, m_rh_lf->bpos);
  }

  //============================================================================
  void visualize(MJDataPtr data, simulation::MarkerManager& markers) override {
    markers.add_cube_marker(
        m_rh_th->xpos_desired, 0.002, math::Vector4d(1, 0, 0, 1));
    markers.add_cube_marker(
        m_rh_ff->xpos_desired, 0.002, math::Vector4d(0, 0, 1, 1));

    std::vector<SubChainControllerPtr> arm_fingers
        = {m_arm, m_rh_th, m_rh_ff, m_rh_mf, m_rh_rf, m_rh_lf};
    for (SubChainControllerPtr finger : arm_fingers) {
      markers.add_frame_marker(finger->xpos_desired, finger->xmat_desired);
      markers.add_frame_marker(
          finger->ee->get_xpos(data, finger->bpos),
          finger->ee->get_xmat(data, finger->rotm),
          0.04);
    }

    // draw contact force
    if (m_simctc_th->is_active()) {
      markers.add_arrow_marker(
          m_simctc_th->get_xpos(data),
          m_simctc_th->get_xpos(data) + m_simctc_th->get_xfrc() * 0.05,
          0.002,
          Vector4d(0, 0, 0, 1));
    }
    if (m_simctc_ff->is_active()) {
      markers.add_arrow_marker(
          m_simctc_ff->get_xpos(data),
          m_simctc_ff->get_xpos(data) + m_simctc_ff->get_xfrc() * 0.05,
          0.002,
          Vector4d(0, 0, 0, 1));
    }
  }

  //============================================================================
  void control(MJDataPtr data) override {
    math::VectorXd ctrl = m_controller->compute(data);
    m_robot->set_ctrl(data, ctrl);
  }

private:
  ConstRigidBodyTreePtr m_robot;

  SubChainControllerPtr m_rh_th;
  SubChainControllerPtr m_rh_ff;
  SubChainControllerPtr m_rh_mf;
  SubChainControllerPtr m_rh_rf;
  SubChainControllerPtr m_rh_lf;
  SubChainControllerPtr m_arm;
  TaskSpaceControllerPtr m_controller;

  simulation::ConstContactPtr m_simctc_th;
  simulation::ConstContactPtr m_simctc_ff;
};

int main(int argc, char const* argv[]) {
  common::ArgumentParser parser(
      "This is a program showing the usage of hybrid controller.");
  parser.ParseCLI(argc, argv);
  MySim sim("../resources/wam_shadow_btsp/wam4_shadow_btsp_box.xml", true);
  sim.run();

  return 0;
}
