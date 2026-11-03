#include "crobot/math/surface.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/math/matrix.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
std::tuple<Vector3d, Vector3d> Surface::tangent(double u, double v) const {
  array2<Vector3d> ptder = derivatives(1, u, v);

  Vector3d dpdu = ptder(1, 0);
  Vector3d dpdv = ptder(0, 1);
  CROBOT_ASSERT_ERROR(
      dpdu.cross(dpdv).norm() < 1e-6,
      "Tangent vectors are aligned to each other.");

  Vector3d t = dpdu.normalized();
  Vector3d b = (dpdv - dpdu.dot(t) * t).normalized();
  return std::make_tuple(t, b.normalized());
}

//==============================================================================
Vector3d Surface::normal(double u, double v) const {
  Vector3d t, b;
  std::tie(t, b) = tangent(u, v);
  return t.cross(b).normalized();
}

//==============================================================================
Matrix32 Surface::normal_jacb(double u, double v) const {
  // array2<Vector3d> ptder = derivatives(2, u, v);
  // Vector3d dpdu          = ptder(1, 0);
  // Vector3d dpdv          = ptder(0, 1);
  // Vector3d d2pduu        = ptder(2, 0);
  // Vector3d d2pdvv        = ptder(0, 2);
  // Vector3d d2pduv        = ptder(1, 1);

  // double dpdu_norm = dpdu.norm();
  // double dpdv_norm = dpdv.norm();

  // Vector3d t = dpdu / dpdu_norm;
  // Vector3d m = dpdv / dpdv_norm;

  // Vector3d dtdu = d2pduu / dpdu_norm - dpdu * dpdu.dot(d2pduu);
  // Vector3d dtdv = d2pduv / dpdu_norm - dpdu * dpdu.dot(d2pduv);
  // Vector3d dmdu = d2pduv / dpdv_norm - dpdv * dpdv.dot(d2pduv);
  // Vector3d dmdv = d2pdvv / dpdv_norm - dpdv * dpdv.dot(d2pdvv);

  // Matrix32 jacb;
  // jacb.col(0) = dmdu.cross(t) + m.cross(dtdu);
  // jacb.col(1) = dmdv.cross(t) + m.cross(dtdv);

  Matrix32 jacb;
  jacb.col(0) = (normal(u + 1e-6, v) - normal(u - 1e-6, v)) / 2e-6;
  jacb.col(1) = (normal(u, v + 1e-6) - normal(u, v - 1e-6)) / 2e-6;

  return jacb;
}

//==============================================================================
Matrix6X Surface::normal_hess(double u, double v) const {
  math::Matrix32 jacb_u
      = (normal_jacb(u + 1e-6, v) - normal_jacb(u - 1e-6, v)) / 2e-6;
  math::Matrix32 jacb_v
      = (normal_jacb(u, v + 1e-6) - normal_jacb(u, v - 1e-6)) / 2e-6;
  math::Matrix6X hess    = math::Matrix6X::Zero(6, 2);
  hess.block<3, 2>(0, 0) = jacb_u;
  hess.block<3, 2>(3, 0) = jacb_v;
  return hess;
}
