#include "crobot/common/common.hpp"
#include "crobot/simulation/simulation.hpp"

using namespace crobot;
using namespace simulation;

int main(int argc, char const* argv[]) {
  common::ArgumentParser parser(
      "This is a program showing the usage of model loader.");
  parser.ParseCLI(argc, argv);

  MJSim sim(CROBOT_RESOURCE_LOCAL_PATH "adept_shadow_btsp/adept_shadow.xml");

  ConstMJRBTreePtr robot = sim.get_rbtree();
  robot->show_details();

  sim.show();

  return 0;
}
