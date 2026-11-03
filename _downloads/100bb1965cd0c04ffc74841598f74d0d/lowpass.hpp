#pragma once

#include "crobot/math/common.hpp"

namespace crobot::math {

/**
 * Low-pass filter.
 */

class LowPass {
public:
  LowPass(double alpha, math::VectorXd& y) {
    this->alpha = math::MatrixXd::Identity(y.size(), y.size()) * alpha;
    this->y     = y;
  }

  static LowPass Frequency(double frequency, double dt, math::VectorXd& y) {
    double rc    = 1 / (2 * constantsd::pi() * frequency);
    double alpha = dt / (rc + dt);
    return LowPass(alpha, y);
  }

  virtual ~LowPass() {
  }

  math::VectorXd& operator()(math::VectorXd& x) {
    CROBOT_ASSERT_ERROR(x.size() == this->y.size());
    return this->y = this->y + this->alpha * (x - this->y);
  }

protected:
private:
  math::MatrixXd alpha;

  math::VectorXd y;
};

} // namespace crobot::math

