#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::math {

using AngleAxisd    = Eigen::AngleAxis<double>;
using EulerAngle    = Vector3d;
using EulerAngleSeq = Eigen::EulerAngleSeq;

/**
 * @brief Wrap angle in radians to [0 2*pi]
 *
 * Wraps angles in LAMBDA, in radians, to the interval [0 2*pi] such that zero
 * maps to zero and 2*pi maps to 2*pi. (In general, positive multiples of 2*pi
 * map to 2*pi and negative multiples of 2*pi map to zero.)
 */
double inline wrap_to_2pi(double lambda) {
  lambda = std::fmod(lambda, 2.0 * EIGEN_PI);
  if (lambda < 0) {
    lambda += 2.0 * EIGEN_PI;
  }
  return lambda;
}

/**
 * @brief Wrap angle in radians to [-pi/2 pi/2]
 *
 */
double inline wrap_to_pi2(double lambda) {
  if (std::abs(lambda) > EIGEN_PI / 2.0) {
    return wrap_to_2pi(lambda + EIGEN_PI / 2.0) - EIGEN_PI / 2.0;
  }
  else {
    return lambda;
  }
}

/**
 * @brief Wrap angle in radians to interval [-pi pi]
 *
 * Positive, odd multiples of pi map to pi and negative, odd multiples of pi map
 * to -pi.
 */
double inline wrap_to_pi(double lambda) {
  if (std::abs(lambda) > EIGEN_PI) {
    return wrap_to_2pi(lambda + EIGEN_PI) - EIGEN_PI;
  }
  else {
    return lambda;
  }
}

/**
 * @brief Log mapping
 *
 * @note When |Log(R)| = PI, Exp(LogR(R) = Exp(-Log(R)). The implementation
 * returns only the positive one.
 */
inline math::Vector3d logMap(const math::Matrix3d& rotm) {
  math::AngleAxisd aaxis(rotm);
  return aaxis.angle() * aaxis.axis();
}

/**
 * @brief Log mapping
 */
inline math::Vector6d logMap(const Eigen::Isometry3d& tform) {
  //--------------------------------------------------------------------------
  // T = (R, p) = exp([w, v]), t = ||w||
  // v = beta*p + gamma*w + 1 / 2*cross(p, w)
  //    , beta = t*(1 + cos(t)) / (2*sin(t)), gamma = <w, p>*(1 - beta) / t^2
  //--------------------------------------------------------------------------
  double theta = std::acos(std::max(
      std::min(0.5 * (tform(0, 0) + tform(1, 1) + tform(2, 2) - 1.0), 1.0),
      -1.0));
  double beta;
  double gamma;
  math::Vector6d ret;

  if (theta > constantsd::pi() - constantsd::epsilon()) {
    const double c1 = 0.10132118364234; // 1 / pi^2
    const double c2 = 0.01507440267955; // 1 / 4 / pi - 2 / pi^3
    const double c3 = 0.00546765085347; // 3 / pi^4 - 1 / 4 / pi^2

    double phi   = constantsd::pi() - theta;
    double delta = 0.5 + 0.125 * phi * phi;

    double w[]
        = {tform(2, 1) > tform(1, 2)
               ? theta * sqrt(1.0 + (tform(0, 0) - 1.0) * delta)
               : -theta * sqrt(1.0 + (tform(0, 0) - 1.0) * delta),
           tform(0, 2) > tform(2, 0)
               ? theta * sqrt(1.0 + (tform(1, 1) - 1.0) * delta)
               : -theta * sqrt(1.0 + (tform(1, 1) - 1.0) * delta),
           tform(1, 0) > tform(0, 1)
               ? theta * sqrt(1.0 + (tform(2, 2) - 1.0) * delta)
               : -theta * sqrt(1.0 + (tform(2, 2) - 1.0) * delta)};

    beta  = 0.25 * theta * (constantsd::pi() - theta);
    gamma = (w[0] * tform(0, 3) + w[1] * tform(1, 3) + w[2] * tform(2, 3))
            * (c1 - c2 * phi + c3 * phi * phi);

    ret << w[0], w[1], w[2],
        beta * tform(0, 3) - 0.5 * (w[1] * tform(2, 3) - w[2] * tform(1, 3))
            + gamma * w[0],
        beta * tform(1, 3) - 0.5 * (w[2] * tform(0, 3) - w[0] * tform(2, 3))
            + gamma * w[1],
        beta * tform(2, 3) - 0.5 * (w[0] * tform(1, 3) - w[1] * tform(0, 3))
            + gamma * w[2];
  }
  else {
    double alpha;
    if (theta > constantsd::epsilon()) {
      alpha = 0.5 * theta / sin(theta);
      beta  = (1.0 + cos(theta)) * alpha;
      gamma = (1.0 - beta) / theta / theta;
    }
    else {
      alpha = 0.5 + 1.0 / 12.0 * theta * theta;
      beta  = 1.0 - 1.0 / 12.0 * theta * theta;
      gamma = 1.0 / 12.0 + 1.0 / 720.0 * theta * theta;
    }

    double w[]
        = {alpha * (tform(2, 1) - tform(1, 2)),
           alpha * (tform(0, 2) - tform(2, 0)),
           alpha * (tform(1, 0) - tform(0, 1))};
    gamma *= w[0] * tform(0, 3) + w[1] * tform(1, 3) + w[2] * tform(2, 3);

    ret << w[0], w[1], w[2],
        beta * tform(0, 3) + 0.5 * (w[2] * tform(1, 3) - w[1] * tform(2, 3))
            + gamma * w[0],
        beta * tform(1, 3) + 0.5 * (w[0] * tform(2, 3) - w[2] * tform(0, 3))
            + gamma * w[1],
        beta * tform(2, 3) + 0.5 * (w[1] * tform(0, 3) - w[0] * tform(1, 3))
            + gamma * w[2];
  }

  return ret;
}

} // namespace crobot::math
