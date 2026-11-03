#pragma once

#include <matplot/matplot.h>

#include "crobot/planning/viewer.hpp"

namespace crobot::planning {

class ViewerSE2 : public Viewer {
public:
  ViewerSE2();

  void draw_qpos(const math::VectorXd& q) override;

  void draw_edge(const math::VectorXd& q1, const math::VectorXd& q2) override;

private:
  matplot::figure_handle m_fig;
  matplot::axes_handle m_ax;
};

} // namespace crobot::planning

