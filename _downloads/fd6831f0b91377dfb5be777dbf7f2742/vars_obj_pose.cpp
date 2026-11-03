#include "crobot/ik/vars_obj_pose.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
ObjectPoseVariables::ObjectPoseVariables(const std::string& name)
  : Variable(7, name) {
  m_bounds = optimization::VecBound(7);
  // position bounds
  for (int i = 0; i < 3; ++i) {
    m_bounds[i] = Bounds(-10.0, 10.0);
  }

  // quaternion bounds
  for (int i = 3; i < 7; ++i) {
    m_bounds[i] = Bounds(-1.0, 1.0);
  }

  m_pose = math::VectorXd(7);
  m_pose << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
}

//==============================================================================
void ObjectPoseVariables::set_values_all(const math::VectorXd& x) {
  for (int i = 0; i < 7; ++i) {
    m_pose[i] = x[i];
  }
};

//==============================================================================
math::VectorXd ObjectPoseVariables::get_values_all() const {
  return m_pose;
};

//==============================================================================
VecBound ObjectPoseVariables::get_bounds_all() const {
  return m_bounds;
}
