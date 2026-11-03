#pragma once

#include <memory>

#include "crobot/ik/variable.hpp"
#include "crobot/math/curve_segment.hpp"

namespace crobot::ik {

class CurveSegmentVariable : public ik::Variable {
public:
  CurveSegmentVariable(
      const std::string& name,
      const std::shared_ptr<const math::CurveSegment>& segment);

  // Documentation inherited
  void set_values_all(const math::VectorXd& x) override;

  // Documentation inherited
  math::VectorXd get_values_all() const override;

  // Documentation inherited
  optimization::VecBound get_bounds_all() const override;

private:
  double m_u;

  std::shared_ptr<const math::CurveSegment> m_segment;

  optimization::VecBound m_bounds;
};

} // namespace crobot::ik

