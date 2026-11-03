EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 3, 3> skew() const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  Matrix<Scalar, 3, 3> res;
  res(0, 0) = 0;
  res(0, 1) = -this->derived().z();
  res(0, 2) = this->derived().y();
  res(1, 0) = this->derived().z();
  res(1, 1) = 0;
  res(1, 2) = -this->derived().x();
  res(2, 0) = -this->derived().y();
  res(2, 1) = this->derived().x();
  res(2, 2) = 0;
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 3, 1> vee() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
  Matrix<Scalar, 3, 1> res;
  res.x() = (this->derived()(2, 1) - this->derived()(1, 2)) / Scalar(2);
  res.y() = (this->derived()(0, 2) - this->derived()(2, 0)) / Scalar(2);
  res.z() = (this->derived()(1, 0) - this->derived()(0, 1)) / Scalar(2);
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 4, 4> rotm2tform() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
  Matrix<Scalar, 4, 4> res       = Matrix<Scalar, 4, 4>::Zero();
  res.template block<3, 3>(0, 0) = this->derived();
  res(3, 3)                      = 1.;
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 4, 4> trvec2tform() const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  Matrix<Scalar, 4, 4> res = Matrix<Scalar, 4, 4>::Identity();
  res(0, 3)                = this->derived().x();
  res(1, 3)                = this->derived().y();
  res(2, 3)                = this->derived().z();
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 3, 3> eul2rotm(
    Eigen::EulerAngleSeq seq = Eigen::EulerAngleSeq::ZYX) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  Matrix<Scalar, 3, 1> ct;
  ct(0) = std::cos(this->derived().x());
  ct(1) = std::cos(this->derived().y());
  ct(2) = std::cos(this->derived().z());
  Matrix<Scalar, 3, 1> st;
  st(0) = std::sin(this->derived().x());
  st(1) = std::sin(this->derived().y());
  st(2) = std::sin(this->derived().z());

  Matrix<Scalar, 3, 3> res;
  if (seq == Eigen::EulerAngleSeq::ZYX) {
    res(0, 0) = ct(1) * ct(0);
    res(0, 1) = st(2) * st(1) * ct(0) - ct(2) * st(0);
    res(0, 2) = ct(2) * st(1) * ct(0) + st(2) * st(0);
    res(1, 0) = ct(1) * st(0);
    res(1, 1) = st(2) * st(1) * st(0) + ct(2) * ct(0);
    res(1, 2) = ct(2) * st(1) * st(0) - st(2) * ct(0);
    res(2, 0) = -st(1);
    res(2, 1) = st(2) * ct(1);
    res(2, 2) = ct(2) * ct(1);
  }
  else if (seq == Eigen::EulerAngleSeq::ZYZ) {
    res(0, 0) = ct(0) * ct(2) * ct(1) - st(0) * st(2);
    res(0, 1) = -ct(0) * ct(1) * st(2) - st(0) * ct(2);
    res(0, 2) = ct(0) * st(1);
    res(1, 0) = st(0) * ct(2) * ct(1) + ct(0) * st(2);
    res(1, 1) = -st(0) * ct(1) * st(2) + ct(0) * ct(2);
    res(1, 2) = st(0) * st(1);
    res(2, 0) = -st(1) * ct(2);
    res(2, 1) = st(1) * st(2);
    res(2, 2) = ct(1);
  }
  else if (seq == Eigen::EulerAngleSeq::XYZ) {
    res(0, 0) = ct(1) * ct(2);
    res(0, 1) = -ct(1) * st(2);
    res(0, 2) = st(1);
    res(1, 0) = ct(0) * st(2) + ct(2) * st(0) * st(1);
    res(1, 1) = ct(0) * ct(2) - st(0) * st(1) * st(2);
    res(1, 2) = -ct(1) * st(0);
    res(2, 0) = st(0) * st(2) - ct(0) * ct(2) * st(1);
    res(2, 1) = ct(2) * st(0) + ct(0) * st(1) * st(2);
    res(2, 2) = ct(0) * ct(1);
  }
  else {
    assert(false);
  }
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 4, 4> eul2tform(
    Eigen::EulerAngleSeq seq = Eigen::EulerAngleSeq::ZYX) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  return this->derived().eul2rotm(seq).rotm2tform();
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 4, 4> tform_inv() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4)
  Matrix<Scalar, 3, 3> rotm
      = this->derived().template block<3, 3>(0, 0).transpose();
  Matrix<Scalar, 3, 1> trvec = this->derived().template block<3, 1>(0, 3);

  Matrix<Scalar, 4, 4> res       = Matrix<Scalar, 4, 4>::Zero();
  res.template block<3, 3>(0, 0) = rotm;
  res.template block<3, 1>(0, 3) = -rotm * trvec;
  res(3, 3)                      = 1;
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 6, 6> tform2xform() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4)
  Matrix<Scalar, 3, 3> rotm  = this->derived().template block<3, 3>(0, 0);
  Matrix<Scalar, 3, 1> trvec = this->derived().template block<3, 1>(0, 3);

  Matrix<Scalar, 6, 6> res       = Matrix<Scalar, 6, 6>::Zero();
  res.template block<3, 3>(0, 0) = rotm;
  res.template block<3, 3>(3, 0) = trvec.skew() * rotm;
  res.template block<3, 3>(3, 3) = rotm;
  return res;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 3, 3> tform2rotm() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4)
  return this->derived().template block<3, 3>(0, 0);
}

EIGEN_DEVICE_FUNC
inline Quaternion<Scalar> tform2quat() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4)
  Matrix<Scalar, 3, 3> rotm = this->derived().template block<3, 3>(0, 0);
  return Quaternion<Scalar>(rotm);
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 3, 1> tform2trvec() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4)
  return this->derived().template block<3, 1>(0, 3);
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 4, 4> tform_flip_normal() const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 4)
  Matrix<Scalar, 3, 3> rotm      = this->derived().template block<3, 3>(0, 0);
  Matrix<Scalar, 3, 1> trvec     = this->derived().template block<3, 1>(0, 3);
  Matrix<Scalar, 4, 4> res       = Matrix<Scalar, 4, 4>::Zero();
  res.template block<3, 1>(0, 0) = rotm.template block<3, 1>(0, 0);
  res.template block<3, 1>(0, 1) = -rotm.template block<3, 1>(0, 1);
  res.template block<3, 1>(0, 2) = -rotm.template block<3, 1>(0, 2);
  res.template block<3, 1>(0, 3) = trvec;
  res(3, 3)                      = 1;
  return res;
}

/**
 * @brief Spatial cross product for motion vectors. NOTE that the motionVec is
 * moving with spatial velocity v.
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 6, 1> cross_motion(
    const MatrixBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6)
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived, 6)
  Matrix<Scalar, 3, 3> sc1 = this->derived().template segment<3>(0).skew();
  Matrix<Scalar, 3, 3> sc2 = this->derived().template segment<3>(3).skew();
  Matrix<Scalar, 6, 6> S;
  S.template block<3, 3>(0, 0) = sc1;
  S.template block<3, 3>(0, 3) = Matrix<Scalar, 3, 3>::Zero();
  S.template block<3, 3>(3, 0) = sc2;
  S.template block<3, 3>(3, 3) = sc1;
  return S * other;
}

/**
 * @brief Spatial cross product for force vectors. NOTE that the forceVec is
 * moving with spatial velocity v.
 */
template <typename OtherDerived>
EIGEN_DEVICE_FUNC inline Matrix<Scalar, 6, 1> cross_force(
    const MatrixBase<OtherDerived>& forceVec) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6)
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived, 6)
  Matrix<Scalar, 3, 3> sc1 = this->derived().template segment<3>(0).skew();
  Matrix<Scalar, 3, 3> sc2 = this->derived().template segment<3>(3).skew();
  Matrix<Scalar, 6, 6> S;
  S.template block<3, 3>(0, 0) = sc1;
  S.template block<3, 3>(0, 3) = sc2;
  S.template block<3, 3>(3, 0) = Matrix<Scalar, 3, 3>::Zero();
  S.template block<3, 3>(3, 3) = sc1;
  return S * forceVec;
}

EIGEN_DEVICE_FUNC inline Matrix<Scalar, Eigen::Dynamic, 1> trim_leading_zeros(
    const RealScalar& prec = NumTraits<Scalar>::dummy_precision()) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, Eigen::Dynamic)
  typename internal::nested_eval<Derived, 1>::type self(derived());
  Matrix<Scalar, Eigen::Dynamic, 1> res;
  for (Index i = 0; i < rows(); ++i) {
    if (this->derived()(i)) {
      if (!internal::isMuchSmallerThan(
              self.coeff(i, 0), static_cast<Scalar>(1), prec)) {
        return self.segment(i, rows() - i);
      }
    }
  }
  return res;
}

EIGEN_DEVICE_FUNC inline RealScalar squaredLength() const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  return squaredNorm();
}

EIGEN_DEVICE_FUNC inline RealScalar length() const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  return norm();
}

EIGEN_DEVICE_FUNC inline RealScalar invLength(void) const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  return fast_inv_sqrt(squaredNorm());
}

EIGEN_DEVICE_FUNC inline Matrix<Scalar, MatrixBase::RowsAtCompileTime - 1, 1>
homo2cart_truncate() const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  return derived().template head<MatrixBase::RowsAtCompileTime - 1>();
}

EIGEN_DEVICE_FUNC inline Matrix<Scalar, MatrixBase::RowsAtCompileTime - 1, 1>
homo2cart() const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  double w = derived().coeff(MatrixBase::RowsAtCompileTime - 1);
  return derived().template head<MatrixBase::RowsAtCompileTime - 1>() / w;
}

EIGEN_DEVICE_FUNC inline Matrix<Scalar, MatrixBase::RowsAtCompileTime + 1, 1>
cart2homo(Scalar weight = 1.0) const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  Matrix<Scalar, MatrixBase::RowsAtCompileTime + 1, 1> homo;
  homo.template head<MatrixBase::RowsAtCompileTime>() = derived() * weight;
  homo.coeffRef(MatrixBase::RowsAtCompileTime)        = weight;
  return homo;
}

EIGEN_DEVICE_FUNC
inline Matrix<Scalar, 3, 3> omega2eulrdot_mat(
    EulerAngleSeq seq = EulerAngleSeq::ZYX) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)

  Matrix<Scalar, 3, 3> res;
  // clang-format off
  if (seq == EulerAngleSeq::ZYX) {
    // Euler angle sequence 1,2,3 in book "Representing attitude" by Diebel.
    // The order of angles is flipped in the textbook.
    // See Eq. 295
    Scalar psi   = derived().x(); // angle corresponds to Z-axis rotation (axis 3)
    Scalar theta = derived().y(); // angle corresponds to Y-axis rotation (axis 2)
    Scalar phi   = derived().z(); // angle corresponds to X-axis rotation (axis 1)
    Scalar cos_theta = std::cos(theta);
    Scalar sin_theta = std::sin(theta);
    Scalar cos_psi = std::cos(psi);
    Scalar sin_psi = std::sin(psi);
    res << 
         sin_theta * cos_psi, sin_theta * sin_psi, cos_theta,
        -cos_theta * sin_psi, cos_theta * cos_psi, Scalar(0),
         cos_psi,             sin_psi,             Scalar(0); 
    res = res / cos_theta;
  }
  else if (seq == EulerAngleSeq::XYZ) {
    // Euler angle sequence 3,2,1
    // See Eq. 457
    Scalar psi   = derived().x(); // angle corresponds to X-axis rotation (axis 1)
    Scalar theta = derived().y(); // angle corresponds to Y-axis rotation (axis 2)
    Scalar phi   = derived().z(); // angle corresponds to Z-axis rotation (axis 3)
    Scalar cos_theta = std::cos(theta);
    Scalar sin_theta = std::sin(theta);
    Scalar cos_psi = std::cos(psi);
    Scalar sin_psi = std::sin(psi);
    res <<  
        cos_theta, sin_theta * sin_psi, -sin_theta * cos_psi,
        Scalar(0), cos_theta * cos_psi,  cos_theta * sin_psi,
        Scalar(0),            -sin_psi,              cos_psi;

    res = res / cos_theta;
  }
  else {
    assert(false);
  }
  // clang-format on
  return res;
}
