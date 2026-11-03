#include "crobot/math/polynomial.hpp"

using namespace crobot;
using namespace math;

//==============================================================================
double crobot::math::polyval(const Eigen::Ref<const VectorXd> p, double x) {
  CROBOT_ASSERT_ERROR(
      p.size() > 0, "Polynomial should have at least one coefficient.");

  double y = p(0);
  for (int i = 1; i < p.size(); ++i) {
    y = y * x + p(i);
  }

  return y;
}

//==============================================================================
double crobot::math::polyval2d(
    const Eigen::Ref<const MatrixXd> c, double x, double y) {
  CROBOT_ASSERT_ERROR(
      c.size() > 0, "Polynomial should have at least one coefficient.");
  int n = c.rows();
  int m = c.cols();

  double f = c(0, 0);
  for (int i = 1; i < n; ++i) {
    f = f * x + c(i, 0);
  }
  for (int j = 1; j < m; ++j) {
    double g = c(0, j);
    for (int i = 1; i < n; ++i) {
      g = g * x + c(i, j);
    }
    f = f * y + g;
  }
  return f;
}

//==============================================================================
VectorXd crobot::math::polyadd(
    const Eigen::Ref<const VectorXd> U, const Eigen::Ref<const VectorXd> V) {
  VectorXd P;
  if (U.rows() > V.rows()) {
    P = U;
    P.segment(U.rows() - V.rows(), V.rows()) += V;
  }
  else {
    P = V;
    P.segment(V.rows() - U.rows(), U.rows()) += U;
  }
  P = P.trim_leading_zeros();
  if (P.size() == 0) {
    P = Vector1d(0.0);
  }
  return P;
}

//==============================================================================
VectorXd crobot::math::polysub(
    const Eigen::Ref<const VectorXd> U, const Eigen::Ref<const VectorXd> V) {
  return crobot::math::polyadd(U, -V);
}

//==============================================================================
MatrixXd crobot::math::conv2(
    const Eigen::Ref<const MatrixXd> P, const Eigen::Ref<const MatrixXd> S) {
  const int m = P.rows(), n = P.cols(), M = S.rows(), N = S.cols();
  MatrixXd C = MatrixXd::Zero(m + M - 1, n + N - 1);

  for (int j = 0; j < m + M - 1; ++j) {
    for (int k = 0; k < n + N - 1; ++k) {
      for (int p = std::max(j - M + 1, 0); p < std::min(m, j + 1); ++p) {
        for (int q = std::max(k - N + 1, 0); q < std::min(n, k + 1); ++q) {
          C(j, k) += P(p, q) * S(j - p, k - q);
        }
      }
    }
  }

  return C;
}

//==============================================================================
VectorXd crobot::math::conv(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v) {
  return conv2(u, v);
}

//==============================================================================
VectorXd crobot::math::polymul(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v) {
  return conv2(u, v);
}

//==============================================================================
VectorXd crobot::math::polyder(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v) {
  int nu = u.size();
  int nv = v.size();

  VectorXd up;
  if (nu < 2) {
    up = Vector1d(0.0);
  }
  else {
    up = (u.topRows(nu - 1).array()
          * VectorXd::LinSpaced(nu - 1, 1, nu - 1).reverse().array())
             .matrix();
  }

  VectorXd vp;
  if (nv < 2) {
    vp = Vector1d(0.0);
  }
  else {
    vp = (v.topRows(nv - 1).array()
          * VectorXd::LinSpaced(nv - 1, 1, nv - 1).reverse().array())
             .matrix();
  }

  VectorXd a1 = conv(up, v);
  VectorXd a2 = conv(u, vp);
  int a1sz    = a1.size();
  int a2sz    = a2.size();
  if (a1sz > a2sz) {
    VectorXd tmp                   = VectorXd::Zero(a1sz);
    tmp.segment(a1sz - a2sz, a2sz) = a2;
    a2                             = tmp;
  }
  else if (a1sz < a2sz) {
    VectorXd tmp                   = VectorXd::Zero(a2sz);
    tmp.segment(a2sz - a1sz, a1sz) = a1;
    a1                             = tmp;
  }

  VectorXd a = a1 + a2;
  if (a.isZero()) {
    a = Vector1d(0.);
  }
  else {
    a = a.trim_leading_zeros();
  }
  VectorXd b = conv(v, v);
  if (b.isZero()) {
    b = Vector1d(0.);
  }
  else {
    b = b.trim_leading_zeros();
  }
  // The vector may be too long when the polynomial coefficients include NaN, so
  // trim the leading element.
  if (a.size() > std::max(nu + nv - 2, 1)) {
    a = a.segment(1, a.rows() - 1);
  }
  return a;
}

//==============================================================================
VectorXd crobot::math::polyder(const Eigen::Ref<const VectorXd> u) {
  return polyder(u, Vector1d(1.0));
}

//==============================================================================
MatrixXd crobot::math::polyder2d(const Eigen::Ref<const MatrixXd> c, int axis) {

  int n = c.rows();
  int m = c.cols();

  if (axis == 0) {
    // treat y as a constant value
    MatrixXd dfdx(n - 1, m);
    for (int i = 0; i < n - 1; ++i) {
      for (int j = 0; j < m; ++j) {
        dfdx(i, j) = c(i, j) * (n - 1 - i);
      }
    }
    return dfdx;
  }
  else if (axis == 1) {
    // treat x as a constant value
    MatrixXd dfdy(n, m - 1);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < m - 1; ++j) {
        dfdy(i, j) = c(i, j) * (m - 1 - j);
      }
    }
    return dfdy;
  }
  else {
    // should not go to this line.
    return MatrixXd();
  }
}

//==============================================================================
VectorXd crobot::math::polyint(const Eigen::Ref<const VectorXd> p, double k) {
  ArrayXd order = ArrayXd::LinSpaced(p.size(), 1, p.size()).reverse();
  VectorXd pi(p.size() + 1);
  pi.segment(0, p.size()) = (p.array() / order).matrix();
  pi(p.size())            = k;
  return pi;
}

//==============================================================================
VectorXd crobot::math::polyfit(
    const Eigen::Ref<const VectorXd> x,
    const Eigen::Ref<const VectorXd> y,
    int order) {
  CROBOT_ASSERT_ERROR(x.size() == y.size());
  CROBOT_ASSERT_ERROR(order >= 1 && order < x.size() - 1);
  // number of input points
  int n = x.size();

  // construct the Vandermonde matrix V = [x.^n ... x.^2 x Ones]
  MatrixXd V(n, order + 1);
  V.col(order) = VectorXd::Ones(n);
  for (int j = order - 1; j >= 0; --j) {
    V.col(j) = (V.col(j + 1).array() * x.array()).matrix();
  }

  // solve least squares problem p = V\y to get polynomial coefficients p.
  VectorXd p = V.householderQr().solve(y);
  return p;
}
