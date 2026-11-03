#pragma once

#include "crobot/planning/rrt_dual.hpp"
namespace crobot::planning {

/**
 * RRT-Connect2.
 *
 * James J. Kuffner, Jr. and Steven M. LaValle. RRT-connect: An efficient
 * approach to single-query path planning. In Proceedings of the IEEE
 * International Conference on Robotics and Automation, pages 995-1001,
 * April 2000.
 *
 * http://dx.doi.org/10.1109/ROBOT.2000.844730
 */
class RRTConnectDual : public RRTDual {
public:
  RRTConnectDual(Planner::ConstStateSpacePtr state_space);

  virtual ~RRTConnectDual();

  std::string get_algorithm_name() const override;

  bool plan(const math::VectorXd& start, const math::VectorXd& goal) override;

protected:
private:
};

} // namespace crobot::planning

