#include "crobot/math/curve.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
Vector3d Curve::tangent(double u) const {
  std::vector<Vector3d> ders = derivatives(1, u);
  Vector3d du                = ders[1];
  double du_len              = du.norm();
  if (!is_zero(du_len)) {
    du /= du_len;
  }
  return du;
}

//==============================================================================
Vector3d Curve::tangent_jacb(double u) const {
  return (tangent(u + 1e-6) - tangent(u - 1e-6)) / 2e-6;
}
