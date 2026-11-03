#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/vars_surface_patch.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
SurfacePatchVariables::SurfacePatchVariables(
    const std::string& name,
    const std::shared_ptr<const math::SurfacePatch>& patch)
  : Variable(2, name), m_patch(patch) {
  m_bounds = optimization::VecBound(2);
  m_bounds.at(0)
      = optimization::Bounds(m_patch->get_lb()[0], m_patch->get_ub()[0]);
  m_bounds.at(1)
      = optimization::Bounds(m_patch->get_lb()[1], m_patch->get_ub()[1]);
  m_uv = math::Vector2d(
      m_bounds[0].lower + (m_bounds[0].upper - m_bounds[0].lower) / 2.,
      m_bounds[1].lower + (m_bounds[1].upper - m_bounds[1].lower) / 2.);
}

//==============================================================================
void SurfacePatchVariables::set_values_all(const math::VectorXd& x) {
  m_uv(0) = x(0);
  m_uv(1) = x(1);
};

//==============================================================================
math::VectorXd SurfacePatchVariables::get_values_all() const {
  return m_uv;
};

//==============================================================================
VecBound SurfacePatchVariables::get_bounds_all() const {
  return m_bounds;
}
