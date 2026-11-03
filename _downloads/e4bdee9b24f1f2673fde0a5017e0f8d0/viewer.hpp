#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::planning {

class Viewer {
public:
  Viewer() = default;

  virtual ~Viewer() = default;

  virtual void draw_qpos(const math::VectorXd& q) = 0;

  virtual void draw_edge(const math::VectorXd& q1, const math::VectorXd& q2)
      = 0;

protected:
private:
};

} // namespace crobot::planning

