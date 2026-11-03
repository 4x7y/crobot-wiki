#pragma once

template <typename Scalar, int Dim, int Mode, int Options>
inline Scalar distance(
    const Transform<Scalar, Dim, Mode, Options>& other,
    const Scalar& weight = 1) const {
  Quaternion<Scalar> q1(rotation());
  Quaternion<Scalar> q2(other.rotation());

  return ::std::sqrt(
      ::std::pow(other(0, 3) - (*this)(0, 3), 2)
      + ::std::pow(other(1, 3) - (*this)(1, 3), 2)
      + ::std::pow(other(2, 3) - (*this)(2, 3), 2)
      + weight * ::std::pow(q1.angularDistance(q2), 2));
}

/**
 * Calculate transformation from a linear and angular velocity vector.
 *
 * @param[in] useApproximation For rotations a, b, c smaller than a
 * few degrees, you can use a faster bi-linear approximation. For
 * rotations larger than 90 degrees, this approximation would be
 * completely wrong.
 */
template <typename Scalar, int Dim, int Mode, int Options>
inline void fromDelta(
    const Transform<Scalar, Dim, Mode, Options>& other,
    const Matrix<Scalar, 6, 1>& delta,
    const bool& useApproximation = false) {
  if (useApproximation) {
    (*this)(0, 0)
        = other(0, 0) - delta(5) * other(1, 0) + delta(4) * other(2, 0);
    (*this)(0, 1)
        = other(0, 1) - delta(5) * other(1, 1) + delta(4) * other(2, 1);
    (*this)(0, 2)
        = other(0, 2) - delta(5) * other(1, 2) + delta(4) * other(2, 2);
    (*this)(0, 3) = other(0, 3) + delta(0);
    (*this)(1, 0)
        = other(1, 0) + delta(5) * other(0, 0) - delta(3) * other(2, 0);
    (*this)(1, 1)
        = other(1, 1) + delta(5) * other(0, 1) - delta(3) * other(2, 1);
    (*this)(1, 2)
        = other(1, 2) + delta(5) * other(0, 2) - delta(3) * other(2, 2);
    (*this)(1, 3) = other(1, 3) + delta(1);
    (*this)(2, 0)
        = other(2, 0) - delta(4) * other(0, 0) + delta(3) * other(1, 0);
    (*this)(2, 1)
        = other(2, 1) - delta(4) * other(0, 1) + delta(3) * other(1, 1);
    (*this)(2, 2)
        = other(2, 2) - delta(4) * other(0, 2) + delta(3) * other(1, 2);
    (*this)(2, 3) = other(2, 3) + delta(2);
    (*this)(3, 0) = 0;
    (*this)(3, 1) = 0;
    (*this)(3, 2) = 0;
    (*this)(3, 3) = 1;
  }
  else {
    Transform<Scalar, Dim, Mode, Options> t;
    t.linear()
        = (AngleAxis<Scalar>(
               delta.segment(3, 3).norm(), delta.segment(3, 3).normalized()))
              .toRotationMatrix();
    t.translation() = t.linear() * delta.segment(0, 3);
    (*this)         = t * other;
  }
}

inline void fromDenavitHartenbergPaul(
    const Scalar& d,
    const Scalar& theta,
    const Scalar& a,
    const Scalar& alpha) {
  Scalar cosAlpha = ::std::cos(alpha);
  Scalar cosTheta = ::std::cos(theta);
  Scalar sinAlpha = ::std::sin(alpha);
  Scalar sinTheta = ::std::sin(theta);

  (*this)(0, 0) = cosTheta;
  (*this)(1, 0) = sinTheta;
  (*this)(2, 0) = Scalar(0);
  (*this)(3, 0) = Scalar(0);
  (*this)(0, 1) = -cosAlpha * sinTheta;
  (*this)(1, 1) = cosAlpha * cosTheta;
  (*this)(2, 1) = sinAlpha;
  (*this)(3, 1) = Scalar(0);
  (*this)(0, 2) = sinAlpha * sinTheta;
  (*this)(1, 2) = -sinAlpha * cosTheta;
  (*this)(2, 2) = cosAlpha;
  (*this)(3, 2) = Scalar(0);
  (*this)(0, 3) = a * cosTheta;
  (*this)(1, 3) = a * sinTheta;
  (*this)(2, 3) = d;
  (*this)(3, 3) = Scalar(1);
}

template <typename Scalar>
inline Matrix<Scalar, 6, 1> getDelta() const {
  Matrix<Scalar, 6, 1> res;

  res(0) = (*this)(0, 3);
  res(1) = (*this)(1, 3);
  res(2) = (*this)(2, 3);
  res(3) = ((*this)(2, 1) - (*this)(1, 2)) / 2.0f;
  res(4) = ((*this)(0, 2) - (*this)(2, 0)) / 2.0f;
  res(5) = ((*this)(1, 0) - (*this)(0, 1)) / 2.0f;

  return res;
}

template <typename Scalar>
inline void setDelta(const Matrix<Scalar, 6, 1>& delta) {
  (*this)(0, 0) = 0;
  (*this)(0, 1) = -delta(5);
  (*this)(0, 2) = delta(4);
  (*this)(0, 3) = delta(0);
  (*this)(1, 0) = delta(5);
  (*this)(1, 1) = 0;
  (*this)(1, 2) = -delta(3);
  (*this)(1, 3) = delta(1);
  (*this)(2, 0) = -delta(4);
  (*this)(2, 1) = delta(3);
  (*this)(2, 2) = 0;
  (*this)(2, 3) = delta(2);
  (*this)(3, 0) = 0;
  (*this)(3, 1) = 0;
  (*this)(3, 2) = 0;
  (*this)(3, 3) = 1;
}

/**
 * Calculate the linear and angular velocity vector between two transformations.
 *
 * @param[in] useApproximation For rotations from t1 to t2 smaller
 * than a few degrees, you can use a faster bi-linear approximation.
 * (For a rotation angle < 1e-3, the approximated angular velocity
 * vector will be accurate up to 1e-9.) For rotations larger than 90
 * degrees, this approximation would be completely wrong.
 */
template <typename Scalar, int Dim, int Mode, int Options>
inline Matrix<Scalar, 6, 1> toDelta(
    const Transform<Scalar, Dim, Mode, Options>& other,
    const bool& useApproximation = false) const {
  Matrix<Scalar, 6, 1> res;

  res(0) = other(0, 3) - (*this)(0, 3);
  res(1) = other(1, 3) - (*this)(1, 3);
  res(2) = other(2, 3) - (*this)(2, 3);

  if (useApproximation) {
    res(3) = ((*this)(1, 0) * other(2, 0) - (*this)(2, 0) * other(1, 0)
              + (*this)(1, 1) * other(2, 1) - (*this)(2, 1) * other(1, 1)
              + (*this)(1, 2) * other(2, 2) - (*this)(2, 2) * other(1, 2))
             / 2.0f;
    res(4) = ((*this)(2, 0) * other(0, 0) - (*this)(0, 0) * other(2, 0)
              + (*this)(2, 1) * other(0, 1) - (*this)(0, 1) * other(2, 1)
              + (*this)(2, 2) * other(0, 2) - (*this)(0, 2) * other(2, 2))
             / 2.0f;
    res(5) = ((*this)(0, 0) * other(1, 0) - (*this)(1, 0) * other(0, 0)
              + (*this)(0, 1) * other(1, 1) - (*this)(1, 1) * other(0, 1)
              + (*this)(0, 2) * other(1, 2) - (*this)(1, 2) * other(0, 2))
             / 2.0f;
  }
  else {
    AngleAxis<Scalar> r(
        Matrix<Scalar, 3, 3>(other.linear() * linear().transpose()));
    res.segment(3, 3) = r.angle() * r.axis();
  }

  return res;
}

inline void toDenavitHartenbergPaul(
    Scalar& d, Scalar& theta, Scalar& a, Scalar& alpha) const {
  assert(::std::abs((*this)(2, 0)) <= ::std::numeric_limits<Scalar>::epsilon());

  d = (*this)(2, 3);

  theta = ::std::atan2((*this)(1, 0), (*this)(0, 0));

  if (::std::abs((*this)(0, 0)) <= ::std::numeric_limits<Scalar>::epsilon()) {
    a = (*this)(1, 3) / (*this)(1, 0);
  }
  else if (
      ::std::abs((*this)(1, 0)) <= ::std::numeric_limits<Scalar>::epsilon()) {
    a = (*this)(0, 3) / (*this)(0, 0);
  }
  else {
    a = (*this)(1, 3) / (*this)(1, 0) + (*this)(0, 3) / (*this)(0, 0);
  }

  alpha = ::std::atan2((*this)(2, 1), (*this)(2, 2));
}

