#include "crobot/planning/viewer_se2.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
ViewerSE2::ViewerSE2() {
  m_fig = matplot::figure(false);
  m_ax  = m_fig->current_axes();
}

//==============================================================================
void ViewerSE2::draw_qpos(const math::VectorXd& q) {
  m_ax->scatter(std::vector<double>{q(0)}, std::vector<double>{q(1)});
  m_fig->draw();
}

//==============================================================================
void ViewerSE2::draw_edge(const math::VectorXd& q1, const math::VectorXd& q2) {
  m_ax->plot({q1(0), q2(0)}, {q1(1), q2(1)}, "b");
  m_fig->draw();
}
