#include <iostream>

#include "crobot/common/common.hpp"
#include "crobot/kdl/kdl.hpp"

using namespace crobot;
using namespace kdl;

int main(int argc, char const* argv[]) {
  common::ArgumentParser parser(
      "This progam shows the details of a given mujoco model.");
  common::HelpFlag help(
      parser, "help", "Show model information.", {'h', "help"});
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

  // Build robot
  RigidBodyTreePtr robot = RigidBodyTree::MakeShared("robot");
  robot->add_body(
      RigidBody::MakeShared(
          "body1", Joint::MakeShared("joint1", JointType::BALL)),
      "world");
  robot->add_body(
      RigidBody::MakeShared(
          "body2", Joint::MakeShared("joint2", JointType::REVOLUTE)),
      "body1");
  robot->add_body(
      RigidBody::MakeShared(
          "body3", Joint::MakeShared("joint3", JointType::PRISMATIC)),
      "body2");
  robot->add_body(
      RigidBody::MakeShared(
          "body4", Joint::MakeShared("joint4", JointType::REVOLUTE)),
      "body2");
  robot->add_body(
      RigidBody::MakeShared(
          "ee", Joint::MakeShared("joint5", JointType::FIXED)),
      "body4");

  // Show robot information
  robot->show_details();

  robot->get_subtree("subtree", "body2")->show_details();

  return 0;
}
