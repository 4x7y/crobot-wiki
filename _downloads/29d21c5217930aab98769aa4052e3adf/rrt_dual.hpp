#pragma once

#include "crobot/planning/rrt.hpp"

namespace crobot::planning {

class RRTDual : public RRT {
public:
  RRTDual(Planner::ConstStateSpacePtr state_space);

  virtual ~RRTDual();

  virtual std::string get_algorithm_name() const override;

  virtual std::vector<math::VectorXd> get_path() override;

  virtual bool plan(
      const math::VectorXd& start, const math::VectorXd& goal) override;
};

} // namespace crobot::planning

