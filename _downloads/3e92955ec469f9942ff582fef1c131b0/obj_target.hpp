#pragma once

#include <functional>

#include "crobot/kdl/kdl.hpp"
#include "crobot/math/math.hpp"

namespace crobot::ik {

class ObjTarget {
public:
  using Ptr      = std::shared_ptr<ObjTarget>;
  using ConstPtr = std::shared_ptr<const ObjTarget>;
  ObjTarget(kdl::ConstRigidBodyTreePtr rbtree, double weight = 0.0)
    : m_rbtree(rbtree), m_weight(weight){};

  ~ObjTarget() = default;

  virtual math::VectorXd get_partial_deriv(
      const math::VectorXd& qpos, const math::VectorXd& qvel) const = 0;

  void set_weight(double weight) {
    m_weight = weight;
  };

  double get_weight() const {
    return m_weight;
  };

protected:
  kdl::ConstRigidBodyTreePtr m_rbtree;

private:
  double m_weight;
};

} // namespace crobot::ik

