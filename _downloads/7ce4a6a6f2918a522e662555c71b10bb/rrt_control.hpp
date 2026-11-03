#pragma once

#include <memory>
#include <vector>

#include "crobot/math/matrix.hpp"
#include "crobot/planning/state_propogator.hpp"

namespace crobot::planning {

struct Control
{
};
using ControlPtr      = std::shared_ptr<Control>;
using ConstControlPtr = std::shared_ptr<const Control>;

struct ControlPath
{
  StatePropagator::Ptr propagator;
  std::vector<math::VectorXd> states;
  std::vector<Control> controls;
};

struct GoalReachFunction
{
  bool is_goal_reached(const math::VectorXd& state);
};
using GoalReachFunctionPtr = std::shared_ptr<GoalReachFunction>;

class RDT {
public:
  using Ptr      = std::shared_ptr<RDT>;
  using ConstPtr = std::shared_ptr<const RDT>;

  // Constuctor that creates a kinodynamic RRT planner from a state propagator
  // object
  RDT(StatePropagator::Ptr propagator, GoalReachFunctionPtr goal_reach_func);

  // Plan a knematically feasible path between two states
  ControlPath plan(const math::VectorXd& start, const math::VectorXd& goal);

private:
  void initialize(const math::VectorXd& start, const math::VectorXd& goal);

  void generate_outputs();

  // Represents the problem-specific planning space
  StatePropagator::Ptr m_state_propagator;

  // Maximum number of nodes allowed in the tree
  int m_max_nodes;

  // Maximum number of iterations
  int m_max_iters;

  // Number times to propagateWhileValid towards goal
  int m_num_goal_extension;

  // Probability of choosing goal state during state sampling
  double m_goal_bias;

  // Call back for determining if goal is reached
  GoalReachFunctionPtr m_goal_reach_func;

  int m_num_nodes;

  int m_num_iters;

  bool m_goal_found;
};

} // namespace crobot::planning

