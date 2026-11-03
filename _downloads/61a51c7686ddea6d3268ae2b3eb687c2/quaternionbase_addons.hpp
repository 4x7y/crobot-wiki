typedef Matrix<Scalar, 4, 4> Matrix4;
typedef Matrix<Scalar, 4, 1> Vector4;

/**
 * @brief Set as the identity quaternion.
 */
EIGEN_DEVICE_FUNC inline void set_identity() {
  setIdentity();
}

/**
 * @brief Convert quaternion to rotation angles.
 */
EIGEN_DEVICE_FUNC inline Vector3 as_euler(
    EulerAngleSeq seq = EulerAngleSeq::ZYX) const {
  Quaternion<Scalar> q = this->normalized();
  return qparts2feul(q.w(), q.x(), q.y(), q.z(), seq);
}

/**
 * @brief Convert quaternion to direction cosine matrix.
 */
EIGEN_DEVICE_FUNC inline Matrix3 as_dcm() const {
  Quaternion<Scalar> q = this->normalized();
  Matrix3 dcm;
  dcm(0, 0) = std::pow(q.w(), 2) + std::pow(q.x(), 2) - std::pow(q.y(), 2)
              - std::pow(q.z(), 2);
  dcm(0, 1) = 2. * (q.x() * q.y() + q.w() * q.z());
  dcm(0, 2) = 2. * (q.x() * q.z() - q.w() * q.y());
  dcm(1, 0) = 2. * (q.x() * q.y() - q.w() * q.z());
  dcm(1, 1) = std::pow(q.w(), 2) - std::pow(q.x(), 2) + std::pow(q.y(), 2)
              - std::pow(q.z(), 2);
  dcm(1, 2) = 2. * (q.y() * q.z() + q.w() * q.x());
  dcm(2, 0) = 2. * (q.x() * q.z() + q.w() * q.y());
  dcm(2, 1) = 2. * (q.y() * q.z() - q.w() * q.x());
  dcm(2, 2) = std::pow(q.w(), 2) - std::pow(q.x(), 2) - std::pow(q.y(), 2)
              + std::pow(q.z(), 2);
  return dcm;
}

/**
 * @brief Convert quaternion to Euler-Rodrigues vector.
 */
EIGEN_DEVICE_FUNC inline Vector3 as_rod() const {
  Quaternion<Scalar> q = this->normalized();
  Scalar thalf         = std::acos(q.w());
  // Rodrigues transformation is not defined for rotation angles equal to +/-pi
  // radians.
  assert(thalf > 1e-18);
  return q.vec() / std::cos(thalf);
}

/**
 * @brief Wrap angle in radians to [0 2*pi]
 *
 * Wraps angles in LAMBDA, in radians, to the interval [0 2*pi] such that zero
 * maps to zero and 2*pi maps to 2*pi. (In general, positive multiples of 2*pi
 * map to 2*pi and negative multiples of 2*pi map to zero.)
 */
static Scalar inline wrap_to_2pi(Scalar lambda) {
  bool positive_input = (lambda > 0);
  lambda              = std::fmod(lambda, 2 * EIGEN_PI);

  if (lambda == 0.0 && positive_input) {
    lambda = 2. * EIGEN_PI;
  }
  return lambda;
}

/**
 * @brief Wrap angle in radians to interval [-pi pi]
 *
 * Positive, odd multiples of pi map to pi and negative, odd multiples of pi map
 * to -pi.
 */
static Scalar inline wrap_to_pi(Scalar lambda) {
  if (std::abs(lambda) > EIGEN_PI) {
    return wrap_to_2pi(lambda + EIGEN_PI) - EIGEN_PI;
  }
  else {
    return lambda;
  }
}

/**
 * @brief Convert quaternion to axis-angle rotation.
 */
EIGEN_DEVICE_FUNC inline AngleAxisType as_axang() const {
  // Normalize the quaternions
  Quaternion<Scalar> q = this->normalized();

  // Normalize and generate the rotation vector and angle sequence
  // For a single quaternion q = [w x y z], the formulas are as follows:
  // (axis) v = [x y z] / norm([x y z]);
  // (angle) theta = 2 * acos(w)
  Vector3 v    = q.vec().normalized();
  Scalar theta = wrap_to_pi(2. * std::acos(q.w()));
  // Handle the zero rotation degenerate case
  // Can return an arbitrary axis, but fix on z-axis rotation
  if (std::fabs(theta) < 1e-18) {
    theta = 0.0;
    v     = Vector3::UnitZ();
  }
  return AngleAxisType(theta, v);
}

/**
 * @brief Convert quaternion to rotation matrix.
 */
EIGEN_DEVICE_FUNC inline Matrix3 as_rotm() const {
  return toRotationMatrix();
}

/**
 * @brief Convert quaternion to homogeneous transformation.
 */
EIGEN_DEVICE_FUNC inline Matrix4 as_tform() const {
  Matrix4 tform           = Matrix4::Identity();
  tform.block(0, 0, 3, 3) = toRotationMatrix();
  return tform;
}

/**
 * @brief Exponential of quaternion.
 *
 * exp(q) is the exponential of the elements of the quaternion q. For
 * quaternion q = a + v where v = bi + cj + dk,
 *
 *   exp(q) = exp(a) * ( cos(||v||) + (v/||v||)*sin(||v||) ).
 */
EIGEN_DEVICE_FUNC inline Quaternion<Scalar> exp() const {
  Scalar vnorm = this->derived().vec().norm();
  Scalar expw  = std::exp(this->derived().w());

  Quaternion<Scalar> q;
  q.w() = expw * std::cos(vnorm);
  if (vnorm != Scalar(0)) {
    Scalar vscale = expw * (std::sin(vnorm) / vnorm);
    q.vec()       = vscale * this->derived().vec();
  }
  else {
    q.vec().setZero();
  }
  return q;
}

/**
 * @brief Natural logarithm of quaternion.
 *
 * log(q) is the natural logarithm of the elements of the quaternion q. For
 * quaternion q = a + v where v = bi + cj + dk,
 *
 *   log(q) = log(||q||) + (v/||v||) * acos(a/||q||).
 */
EIGEN_DEVICE_FUNC inline Quaternion<Scalar> log() const {
  Scalar theta    = ::acos(this->derived().w());
  Scalar sinTheta = ::sin(theta);

  Quaternion<Scalar> q;

  if (sinTheta > Scalar(0)) {
    q.vec() = theta * this->derived().vec() / sinTheta;
  }
  else {
    q.vec().setZero();
  }

  q.w() = Scalar(0);

  return q;
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Quaternion<Scalar> integrated_bodyvel(
    const MatrixBase<OtherDerived>& omega, Scalar dt) const {
  Quaternion<Scalar> quatPert(
      0, 0.5 * dt * omega(0), 0.5 * dt * omega(1), 0.5 * dt * omega(2));
  return this->derived() * quatPert.exp();
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Quaternion<Scalar> integrated_worldvel(
    const MatrixBase<OtherDerived>& omega, Scalar dt) const {
  Quaternion<Scalar> quatPert(
      0, 0.5 * dt * omega(0), 0.5 * dt * omega(1), 0.5 * dt * omega(2));
  return quatPert.exp() * this->derived();

  // return (this->derived() * quatPert + this->derived()).normalized();
}

/**
 * @brief Add two quaternions together.
 */
template <typename OtherDerived>
Quaternion<Scalar> operator+(const QuaternionBase<OtherDerived>& other) const {
  return Quaternion<Scalar>(this->derived().coeffs() + other.coeffs());
}

/**
 * @brief Subtract quaternions, express as 3D velocity:
 *
 *              quat(res) = quatThis.inv() * quatOther.
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Vector3 subtract(
    const QuaternionBase<OtherDerived>& other) const {
  Quaternion<Scalar> diff = this->derived().conjugate() * other;
  AngleAxisType aaxis     = diff.as_axang();
  Vector3 omega           = aaxis.angle() * aaxis.axis();
  return omega;
}

/** @brief Return a vector expression of (w,x,y,z) */
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 4, 1> as_vector() const {
  Matrix<Scalar, 4, 1> vec(
      this->derived().w(),
      this->derived().x(),
      this->derived().y(),
      this->derived().z());
  return vec;
}

/**
 * @brief Compute time-derivative of quaternion, given 3D rotational velocity.
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 4, 1> derivative(
    const MatrixBase<OtherDerived>& omega) const {
  Quaternion<Scalar> quat_omega(0, omega(0), omega(1), omega(2));
  return Scalar(0.5) * (quat_omega * this->derived()).as_vector();
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 1> angular_velocity(
    const MatrixBase<OtherDerived>& quat_deriv) const {
  Quaternion<Scalar> quatdot(
      quat_deriv(0), quat_deriv(1), quat_deriv(2), quat_deriv(3));
  return 2. * (quatdot * this->derived().conjugate()).vec();
}

/**
 * @brief A mapping matrix that converts the derivative of a quaternion to the
 * corresponding angular velocity.
 *
 * @tparam OtherDerived
 * @return EIGEN_DEVICE_FUNC
 */
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4> mat_quatderiv2omega() {
  Matrix<Scalar, 3, 4> mat;
  Scalar q0 = derived().w();
  Scalar q1 = derived().x();
  Scalar q2 = derived().y();
  Scalar q3 = derived().z();

  // clang-format off
  mat << -q1,  q0,  q3, -q2, //
         -q2, -q3,  q0,  q1, //
         -q3,  q2, -q1,  q0; //
  // clang-format on

  return 2.0 * mat;
}

/**
 * @brief A mapping matrix that converts an angular velocity to the
 * corresponding quaternion derivative.
 *
 * @tparam OtherDerived
 * @return EIGEN_DEVICE_FUNC
 */
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 4, 3> mat_omega2quatderiv() {
  Matrix<Scalar, 4, 3> mat;
  Scalar q0 = derived().w();
  Scalar q1 = derived().x();
  Scalar q2 = derived().y();
  Scalar q3 = derived().z();

  // clang-format off
  mat << -q1, -q2, -q3,
          q0,  q3, -q2,
         -q3,  q0,  q1,
          q2, -q1,  q0;
  // clang-format on

  return 0.5 * mat;
}

/**
 * @brief Subtract one quaternion from the other coefficient-wise.
 */
template <typename OtherDerived>
Quaternion<Scalar> operator-(const QuaternionBase<OtherDerived>& other) const {
  return Quaternion<Scalar>(this->derived().coeffs() - other.coeffs());
}

Quaternion<Scalar> operator*(const Scalar& scalar) const {
  return Quaternion<Scalar>(this->derived().coeffs() * scalar);
}

Quaternion<Scalar> operator/(const Scalar& scalar) const {
  return Quaternion<Scalar>(this->derived().coeffs() / scalar);
}

Quaternion<Scalar> pow(const Scalar& t) const {
  return (this->derived().log() * Scalar(t)).exp();
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Vector3 rotate(
    const MatrixBase<OtherDerived>& vec) const {
  Scalar q0  = this->derived().w();
  Vector3 qv = this->derived().vec();

  return (q0 * q0 - qv.dot(qv)) * vec + 2 * qv.dot(vec) * qv
         + 2 * q0 * qv.cross(vec);
}

/**
 * @brief Computes the partial derivatives of a vector rotated by a quaternion,
 * i.e., v' = q ⊗ v ⊗ q.conjugate(), with respect to the quaternion components
 * [qx, qy, qz, qw]. The result is a 3x4 Jacobian matrix where each column
 * corresponds to the derivative of the rotated vector with respect to one
 * quaternion coefficient.
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4> diff_rotvec_coeffs(
    const MatrixBase<OtherDerived>& vec) const {
  Scalar q0  = this->derived().w();
  Vector3 qv = this->derived().vec();

  Matrix<Scalar, 3, 4> diff;
  diff.template block<3, 3>(0, 0) = -vec * qv.transpose()
                                    + vec.dot(qv) * Matrix3::Identity()
                                    + qv * vec.transpose() - q0 * vec.skew();
  diff.template block<3, 1>(0, 3) = q0 * vec + qv.cross(vec);

  return 2 * diff;
}

/**
 * @brief Same as diff_rotvec_coeffs, but with the quaternion coefficients
 * ordered as [qw, qx, qy, qz] instead of the Eigen convention [qx, qy, qz, qw].
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4> diff_rotvec_coeffs_wxyz(
    const MatrixBase<OtherDerived>& vec) const {
  const Matrix<Scalar, 3, 4> J1 = this->diff_rotvec_coeffs(vec);
  Matrix<Scalar, 3, 4> J2;
  J2.template block<3, 1>(0, 0) = J1.template block<3, 1>(0, 3);
  J2.template block<3, 3>(0, 1) = J1.template block<3, 3>(0, 0);
  return J2;
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4> nonunit_diff_rotvec_coeffs(
    const MatrixBase<OtherDerived>& vec) const {

  return this->normalized().diff_rotvec_coeffs(vec) * this->unit_deriv();
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4>
nonunit_diff_conjugate_rotvec_coeffs(
    const MatrixBase<OtherDerived>& vec) const {

  return this->normalized().diff_conjugate_rotvec_coeffs(vec)
         * this->unit_deriv();
}

template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4> diff_conjugate_rotvec_coeffs(
    const MatrixBase<OtherDerived>& vec) const {
  // Matrix4 scale                    = Matrix4::Identity();
  // scale.template block<3, 3>(1, 1) = -Matrix3::Identity();
  // return this->derived().conjugate().diff_rotvec_coeffs(vec) * scale;

  Scalar q0  = this->derived().w();
  Vector3 qv = this->derived().vec();

  Matrix<Scalar, 3, 4> diff;
  diff.template block<3, 1>(0, 0) = q0 * vec - qv.cross(vec);
  diff.template block<3, 3>(0, 1) = -vec * qv.transpose()
                                    + vec.dot(qv) * Matrix3::Identity()
                                    + qv * vec.transpose() + q0 * vec.skew();

  return 2 * diff;
}

/**
 * @brief Partial derivative of normalized q w.r.t q , i.e. d(q/||q||)/d(q)
 */
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 4, 4> unit_deriv() const {

  Vector4 q(
      this->derived().w(),
      this->derived().x(),
      this->derived().y(),
      this->derived().z());
  Scalar qnorm = q.norm();

  Matrix4 dq_dq = Matrix4::Identity() - q * q.transpose() / (qnorm * qnorm);
  return dq_dq / qnorm;
}

/**
 * @brief partial (this * other) / partial other quat
 */
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 4, 4> diff_times_other_coeffs() const {
  Scalar p0  = this->derived().w();
  Vector3 pv = this->derived().vec();

  Matrix4 diff = Matrix4::Zero();
  diff.template block<1, 4>(0, 0) << p0, -pv.transpose();
  diff.template block<3, 1>(1, 0) = pv;
  diff.template block<3, 3>(1, 1) << p0 * Matrix3::Identity() + pv.skew();

  return diff;
}

/**
 * @brief partial (other * this) / partial other quat
 */
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 4, 4> diff_other_times_coeffs() const {
  Scalar q0  = this->derived().w();
  Vector3 qv = this->derived().vec();

  Matrix4 diff = Matrix4::Zero();
  diff.template block<1, 4>(0, 0) << q0, -qv.transpose();
  diff.template block<3, 1>(1, 0) = qv;
  diff.template block<3, 3>(1, 1) << q0 * Matrix3::Identity() - qv.skew();

  return diff;
}

/**
 * @brief Paritial derivatives of the vector obtained after a quaternion
 * rotation, with respect to the four quaternion coefficients [x y z w].
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 1, 4> diff_angledist_coeffs(
    const QuaternionBase<OtherDerived>& r) const {

  Scalar qw = this->derived().w();
  Scalar qx = this->derived().x();
  Scalar qy = this->derived().y();
  Scalar qz = this->derived().z();

  Scalar rw = r.w();
  Scalar rx = r.x();
  Scalar ry = r.y();
  Scalar rz = r.z();

  // u = q * conj(r)
  Matrix<Scalar, 4, 4> du_dq;
  du_dq.template block<4, 1>(0, 0) = Vector4(rx, rw, rz, -ry);   // du_dqx
  du_dq.template block<4, 1>(0, 1) = Vector4(ry, -rz, rw, rx);   // du_dqy
  du_dq.template block<4, 1>(0, 2) = Vector4(rz, ry, -rx, rw);   // du_dqz
  du_dq.template block<4, 1>(0, 3) = Vector4(rw, -rx, -ry, -rz); // du_dqw

  Quaternion<Scalar> u = this->derived() * r.conjugate();
  Scalar uw            = u.w();
  Scalar ux            = u.x();
  Scalar uy            = u.y();
  Scalar uz            = u.z();

  // n = norm([ux uy uz])
  Scalar n      = u.vec().norm();
  Scalar unorm2 = u.squaredNorm();
  Scalar uwsign = (uw > 0) - (uw < 0);

  // angle = 2 * atan2(n, abs(uw))
  double dm_dn  = 2. * std::abs(uw) / unorm2;
  double dm_duw = -2. * n * uwsign / unorm2;
  double dn_dux = ux / n;
  double dn_duy = uy / n;
  double dn_duz = uz / n;

  Matrix<Scalar, 1, 4> dm_du;
  dm_du(0) = dm_duw;
  dm_du(1) = dm_dn * dn_dux;
  dm_du(2) = dm_dn * dn_duy;
  dm_du(3) = dm_dn * dn_duz;

  Matrix<Scalar, 1, 4> diff = dm_du * du_dq;
  return diff;
}

EIGEN_DEVICE_FUNC inline Matrix<Scalar, 3, 4> diff_omega_coeffs() const {
  Scalar q0 = this->derived().w();
  Scalar q1 = this->derived().x();
  Scalar q2 = this->derived().y();
  Scalar q3 = this->derived().z();

  Matrix<Scalar, 3, 4> J_r;
  J_r.template block<3, 1>(0, 0) = Vector3(-q1, -q2, -q3);
  J_r.template block<3, 1>(0, 1) = Vector3(q0, q3, -q2);
  J_r.template block<3, 1>(0, 2) = Vector3(-q3, q0, q1);
  J_r.template block<3, 1>(0, 3) = Vector3(q2, -q1, q0);
  return 2 * J_r;
}

/**
 * @brief angular distance between two quaternions.
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline double angular_distance(
    const QuaternionBase<OtherDerived>& other) const {
  Scalar q0 = this->derived().w();
  Scalar q1 = this->derived().x();
  Scalar q2 = this->derived().y();
  Scalar q3 = this->derived().z();

  Scalar r0 = other.w();
  Scalar r1 = other.x();
  Scalar r2 = other.y();
  Scalar r3 = other.z();

  double dot = q0 * r0 + q1 * r1 + q2 * r2 + q3 * r3;
  return 2. * std::acos(std::abs(dot));
}

/**
 * @brief Compute the instantaneous first derivative
 *
 * This helper uses the analytical representation of the interpolation
 * method to compute the first derivative of the interpolation method at
 * an instant specified by the time derivative of the time scaling.
 */
template <typename OtherDerived>
Quaternion<Scalar> slerp_first_derivative(
    const Scalar& t,
    const QuaternionBase<OtherDerived>& other,
    const QuaternionBase<OtherDerived>& qinterp) const {
  return qinterp * (this->derived().conjugate() * other).log() * t;
}

/**
 * @brief Compute the instantaneous second derivative.
 *
 * This helper uses the analytical representation of the interpolation
 * method to compute the second derivative of the interpolation method at
 * an instant specified by the second time derivative of the time scaling.
 */
template <typename OtherDerived>
Quaternion<Scalar> slerp_second_derivative(
    const Scalar& t,
    const QuaternionBase<OtherDerived>& other,
    const QuaternionBase<OtherDerived>& qinterp) const {
  return qinterp * (this->derived().conjugate() * other).log()
         * (this->derived().conjugate() * other).log() * t;
}

EIGEN_DEVICE_FUNC static inline Vector3 qparts2feul(
    Scalar qa, Scalar qb, Scalar qc, Scalar qd, EulerAngleSeq rot) {

  bool found = true;
  Scalar tmp, a, b, c;
  switch (rot) {
    case EulerAngleSeq::YZY:
      tmp = std::pow(qa, 2) * 2. - 1. + std::pow(qc, 2) * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::acos(tmp);
      a   = std::atan2(
          (qa * qb * 2. + qc * qd * 2.), (qa * qd * 2. - qb * qc * 2.));
      c = -std::atan2(
          (qa * qb * 2. - qc * qd * 2.), (qa * qd * 2. + qb * qc * 2.));
      break;
    case EulerAngleSeq::YXY:
      tmp = std::pow(qa, 2) * 2. - 1. + std::pow(qc, 2) * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::acos(tmp);
      a   = -std::atan2(
          (qa * qd * 2. - qb * qc * 2.), (qa * qb * 2. + qc * qd * 2.));
      c = std::atan2(
          (qa * qd * 2. + qb * qc * 2.), (qa * qb * 2. - qc * qd * 2.));
      break;
    case EulerAngleSeq::ZYZ:
      tmp = std::pow(qa, 2) * 2. - 1. + std::pow(qd, 2) * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::acos(tmp);
      a   = -std::atan2(
          (qa * qb * 2. - qc * qd * 2.), (qa * qc * 2. + qb * qd * 2.));
      c = std::atan2(
          (qa * qb * 2. + qc * qd * 2.), (qa * qc * 2. - qb * qd * 2.));
      break;
    case EulerAngleSeq::ZXZ:
      tmp = std::pow(qa, 2) * 2. - 1. + std::pow(qd, 2) * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::acos(tmp);
      a   = std::atan2(
          (qa * qc * 2. + qb * qd * 2.), (qa * qb * 2. - qc * qd * 2.));
      c = -atan2((qa * qc * 2. - qb * qd * 2.), (qa * qb * 2. + qc * qd * 2.));
      break;
    case EulerAngleSeq::XYX:
      tmp = std::pow(qa, 2) * 2. - 1. + std::pow(qb, 2) * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::acos(tmp);
      a   = std::atan2(
          (qa * qd * 2. + qb * qc * 2.), (qa * qc * 2. - qb * qd * 2.));
      c = -atan2((qa * qd * 2. - qb * qc * 2.), (qa * qc * 2. + qb * qd * 2.));
      break;
    case EulerAngleSeq::XZX:
      tmp = std::pow(qa, 2) * 2. - 1. + std::pow(qb, 2) * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::acos(tmp);
      a   = -std::atan2(
          (qa * qc * 2. - qb * qd * 2.), (qa * qd * 2. + qb * qc * 2.));
      c = std::atan2(
          (qa * qc * 2. + qb * qd * 2.), (qa * qd * 2. - qb * qc * 2.));
      break;
    case EulerAngleSeq::XYZ:
      tmp = qa * qc * 2. + qb * qd * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::asin(tmp);
      a   = std::atan2(
          (qa * qb * 2. - qc * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qd, 2) * 2.));
      c = std::atan2(
          (qa * qd * 2. - qb * qc * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qb, 2) * 2.));
      break;
    case EulerAngleSeq::YZX:
      tmp = qa * qd * 2. + qb * qc * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::asin(tmp);
      a   = std::atan2(
          (qa * qc * 2. - qb * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qb, 2) * 2.));
      c = std::atan2(
          (qa * qb * 2. - qc * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qc, 2) * 2.));
      break;
    case EulerAngleSeq::ZXY:
      tmp = qa * qb * 2. + qc * qd * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = std::asin(tmp);
      a   = std::atan2(
          (qa * qd * 2. - qb * qc * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qc, 2) * 2.));
      c = std::atan2(
          (qa * qc * 2. - qb * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qd, 2) * 2.));
      break;
    case EulerAngleSeq::XZY:
      tmp = qb * qc * 2. - qa * qd * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = -std::asin(tmp);
      a   = std::atan2(
          (qa * qb * 2. + qc * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qc, 2) * 2.));
      c = std::atan2(
          (qa * qc * 2. + qb * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qb, 2) * 2.));
      break;
    case EulerAngleSeq::ZYX:
      tmp = qb * qd * 2. - qa * qc * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = -std::asin(tmp);
      a   = std::atan2(
          (qa * qd * 2. + qb * qc * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qb, 2) * 2.));
      c = std::atan2(
          (qa * qb * 2. + qc * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qd, 2) * 2.));
      break;
    case EulerAngleSeq::YXZ:
      tmp = qc * qd * 2. - qa * qb * 2.;
      tmp = tmp > +1. ? +1. : tmp;
      tmp = tmp < -1. ? -1. : tmp;
      b   = -std::asin(tmp);
      a   = std::atan2(
          (qa * qc * 2. + qb * qd * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qd, 2) * 2.));
      c = std::atan2(
          (qa * qd * 2. + qb * qc * 2.),
          (std::pow(qa, 2) * 2. - 1. + std::pow(qc, 2) * 2.));
      break;
    default:
      found = false;
      a     = 0.;
      b     = 0.;
      c     = 0.;
  }
  assert(found);
  return Vector3(a, b, c);
}
