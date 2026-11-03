#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/vars_curve_segment.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
CurveSegmentVariable::CurveSegmentVariable(
    const std::string& name,
    const std::shared_ptr<const math::CurveSegment>& segment)
  : Variable(1, name), m_segment(segment) {
  m_bounds = optimization::VecBound(1);
  m_bounds.at(0)
      = optimization::Bounds(m_segment->get_lb(), m_segment->get_ub());
  m_u = m_bounds[0].lower + (m_bounds[0].upper - m_bounds[0].lower) / 2.;
}

//==============================================================================
void CurveSegmentVariable::set_values_all(const math::VectorXd& x) {
  m_u = x(0);
};

//==============================================================================
math::VectorXd CurveSegmentVariable::get_values_all() const {
  return math::Vector1d(m_u);
};

//==============================================================================
VecBound CurveSegmentVariable::get_bounds_all() const {
  return m_bounds;
}
