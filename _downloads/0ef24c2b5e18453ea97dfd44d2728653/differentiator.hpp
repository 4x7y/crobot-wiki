#pragma once

#include "crobot/math/common.hpp"

namespace crobot::math {

/**
 * Low-pass filter.
 */

class Differentiator {
public:
  using Ptr      = std::shared_ptr<Differentiator>;
  using ConstPtr = std::shared_ptr<const Differentiator>;

  Differentiator(double dt, VectorXd& x)
    : dt_(dt),
      x_(x),
      v_(VectorXd::Zero(x.size())),
      a_(VectorXd::Zero(x.size())),
      v_prev_(VectorXd::Zero(x.size())) {
  }

  virtual ~Differentiator() {
  }

  void reset(VectorXd& x) {
    x_ = x;
    v_.setZero();
    a_.setZero();
  }

  std::pair<VectorXd, VectorXd> update(const VectorXd& x) {
    // Calculate velocity
    v_ = (x - x_) / dt_;

    // Calculate acceleration
    a_ = (v_ - v_prev_) / dt_;

    // Store current position and velocity
    x_      = x;
    v_prev_ = v_;

    return std::make_pair(v_, a_);
  }

  VectorXd get_velocity() const {
    return v_;
  }

  VectorXd get_acceleration() const {
    return a_;
  }

private:
  double dt_;
  VectorXd x_;
  VectorXd v_;
  VectorXd a_;
  VectorXd v_prev_;
};

} // namespace crobot::math

