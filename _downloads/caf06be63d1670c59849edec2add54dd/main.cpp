#include <iostream>
#include <string>
#include <thread>

#include "crobot/common/common.hpp"
#include "crobot/ik/gik.hpp"
#include "crobot/kdl/kdl.hpp"
#include "crobot/math/math.hpp"
#include "crobot/simulation/simulation.hpp"

using namespace crobot;
using namespace simulation;

int main(int /* argc */, char const* /* argv */[]) {
  MJSim sim(CROBOT_RESOURCE_LOCAL_PATH "adept_shadow_btsp/adept_shadow.xml");
  ConstMJRBTreePtr robot = sim.get_rbtree();
  robot->show_details();

  ik::GeneralizedIK gik(robot, true);
  gik.add_surface_contact_constraint(
      "surf_contact",
      "rh_ffdistal",
      std::make_shared<math::SurfacePatch>(
          std::make_shared<math::Sphere>(0.01),
          math::Vector2d(0.0, M_PI * 2.0),
          math::Vector2d(0.0, M_PI),
          math::Vector3d(0, -0.0018, 0.0205)),
      "ball",
      std::make_shared<math::SurfacePatch>(
          std::make_shared<math::Sphere>(0.025),
          math::Vector2d(0.0, M_PI * 2.0),
          math::Vector2d(0.0, M_PI / 2.0)),
      0.0001);
  bool solved = gik.solve();
  if (solved) {
    std::cout << "SOLVED" << std::endl;
  }
  else {
    std::cout << "NOT SOLVED" << std::endl;
  }

  robot->set_qpos_mj(gik.get_qpos());
  robot->forward_kinematics_mj();

  sim.show();
  return 0;
}
