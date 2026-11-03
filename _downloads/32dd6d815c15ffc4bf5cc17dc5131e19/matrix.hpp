#pragma once
// clang-format off
#include "crobot/math/detail/eigen_addons.hpp"
#ifndef EIGEN_MATRIXBASE_PLUGIN
#  define EIGEN_MATRIXBASE_PLUGIN "crobot/math/detail/matrixbase_addons.hpp"
#endif
#ifndef EIGEN_MATRIX_PLUGIN
#  define EIGEN_MATRIX_PLUGIN "crobot/math/detail/matrix_addons.hpp"
#endif
#ifndef EIGEN_TRANSFORM_PLUGIN
#  define EIGEN_TRANSFORM_PLUGIN "crobot/math/detail/transform_addons.hpp"
#endif
#ifndef EIGEN_QUATERNIONBASE_PLUGIN
#  define EIGEN_QUATERNIONBASE_PLUGIN "crobot/math/detail/quaternionbase_addons.hpp"
#endif
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "crobot/math/common.hpp"
// clang-format on

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------

namespace crobot::math {

using Scalar  = double;
using Float   = float;
using Integer = int;

template <typename T, int Rows, int Cols>
using Matrix = Eigen::Matrix<T, Rows, Cols, Eigen::RowMajor>;
template <int Dim>
using MatrixNd = math::Matrix<Scalar, Dim, Dim>;
template <int Dim>
using MatrixNi = math::Matrix<Integer, Dim, Dim>;
template <int Rows>
using VectorNd = Eigen::Matrix<Scalar, Rows, 1>;
template <int Rows>
using VectorNi = Eigen::Matrix<Integer, Rows, 1>;

using Vector1f = Eigen::Matrix<Float, 1, 1>;
using Vector1d = Eigen::Matrix<Scalar, 1, 1>;
using Vector1i = Eigen::Matrix<Integer, 1, 1>;
using Vector2f = Eigen::Matrix<Float, 2, 1>;
using Vector2d = Eigen::Matrix<Scalar, 2, 1>;
using Vector2i = Eigen::Matrix<Integer, 2, 1>;
using Vector3f = Eigen::Matrix<Float, 3, 1>;
using Vector3d = Eigen::Matrix<Scalar, 3, 1>;
using Vector3i = Eigen::Matrix<Integer, 3, 1>;
using Vector4f = Eigen::Matrix<Float, 4, 1>;
using Vector4d = Eigen::Matrix<Scalar, 4, 1>;
using Vector4i = Eigen::Matrix<Integer, 4, 1>;
using Vector5d = Eigen::Matrix<Scalar, 5, 1>;
using Vector5i = Eigen::Matrix<Integer, 5, 1>;
using Vector6d = Eigen::Matrix<Scalar, 6, 1>;
using Vector6i = Eigen::Matrix<Integer, 6, 1>;
using VectorXd = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXf = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using VectorXi = Eigen::Matrix<Integer, Eigen::Dynamic, 1>;

using RowVector1d = math::Matrix<Scalar, 1, 1>;
using RowVector1i = math::Matrix<Integer, 1, 1>;
using RowVector2d = math::Matrix<Scalar, 1, 2>;
using RowVector2i = math::Matrix<Integer, 1, 2>;
using RowVector3d = math::Matrix<Scalar, 1, 3>;
using RowVector3i = math::Matrix<Integer, 1, 3>;
using RowVector4d = math::Matrix<Scalar, 1, 4>;
using RowVector4i = math::Matrix<Integer, 1, 4>;
using RowVector5d = math::Matrix<Scalar, 1, 5>;
using RowVector5i = math::Matrix<Integer, 1, 5>;
using RowVector6d = math::Matrix<Scalar, 1, 6>;
using RowVector6i = math::Matrix<Integer, 1, 6>;
using RowVectorXd = math::Matrix<Scalar, 1, Eigen::Dynamic>;
using RowVectorXi = math::Matrix<Integer, 1, Eigen::Dynamic>;

using Matrix1d = math::Matrix<Scalar, 1, 1>;
using Matrix1X = math::Matrix<Scalar, 1, Eigen::Dynamic>;
using Matrix2d = math::Matrix<Scalar, 2, 2>;
using Matrix2f = math::Matrix<Float, 2, 2>;
using Matrix23 = math::Matrix<Scalar, 2, 3>;
using Matrix2X = math::Matrix<Scalar, 2, Eigen::Dynamic>;
using Matrix3d = math::Matrix<Scalar, 3, 3>;
using Matrix3f = math::Matrix<Float, 3, 3>;
using Matrix32 = math::Matrix<Scalar, 3, 2>;
using Matrix34 = math::Matrix<Scalar, 3, 4>;
using Matrix36 = math::Matrix<Scalar, 3, 6>;
using Matrix3X = math::Matrix<Scalar, 3, Eigen::Dynamic>;
using Matrix4f = math::Matrix<Float, 4, 4>;
using Matrix4d = math::Matrix<Scalar, 4, 4>;
using Matrix4i = math::Matrix<Integer, 4, 4>;
using Matrix43 = math::Matrix<Scalar, 4, 3>;
using Matrix4X = math::Matrix<Scalar, 4, Eigen::Dynamic>;
using Matrix5d = math::Matrix<Scalar, 5, 5>;
using Matrix5X = math::Matrix<Scalar, 5, Eigen::Dynamic>;
using Matrix63 = math::Matrix<Scalar, 6, 3>;
using Matrix6d = math::Matrix<Scalar, 6, 6>;
using Matrix6X = math::Matrix<Scalar, 6, Eigen::Dynamic>;
using MatrixX3 = math::Matrix<Scalar, Eigen::Dynamic, 3>;
using MatrixX6 = math::Matrix<Scalar, Eigen::Dynamic, 6>;
using MatrixXd = math::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXf = math::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;

using ArrayXd = Eigen::ArrayXd;

// multibody
using SpatialVector   = Vector6d;
using SpatialMatrix   = Matrix6d;
using Inertia         = Matrix3d;
using LinearJacobian  = Matrix3X;
using AngularJacobian = Matrix3X;
using Jacobian        = Matrix6X;

using Transform        = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using AffineTransform  = Eigen::Transform<Scalar, 3, Eigen::Affine>;
using ProjectTransform = Eigen::Transform<Scalar, 3, Eigen::Projective>;

// optimization
using SparseJacobian = Eigen::SparseMatrix<Scalar, Eigen::RowMajor>;

// gui
using Index           = uint32_t;
using Triangles       = math::Matrix<Index, Eigen::Dynamic, 3>;
using Points          = math::Matrix<Float, Eigen::Dynamic, 3>;
using PointsRGBNormal = math::Matrix<Float, Eigen::Dynamic, 9>;
using PointsRGB       = math::Matrix<Float, Eigen::Dynamic, 6>;
using Points2D        = math::Matrix<Float, Eigen::Dynamic, 2>;
using Image           = math::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;
using ImageU          = math::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

using MapToVectorXd = Eigen::Map<math::VectorXd>;

inline Vector6d compose(const Vector3d& angular, const Vector3d& linear) {
  Vector6d composition;
  composition << angular, linear;
  return composition;
}

inline VectorXd compose(const VectorXd& vec1, const VectorXd& vec2) {
  VectorXd composition(vec1.size() + vec2.size());
  composition << vec1, vec2;
  return composition;
}

/**
 * @brief Calculate Jacobian matrix inverse.
 *
 * @param jacb  Jacobian matrix
 * @param lambda Damping factor
 * @param svd Use singular value decomposition or damped least squares
 * @return math::MatrixXd Inverse of the Jacobian matrix.
 */
inline MatrixXd calculate_jacb_inv(
    const MatrixXd& jacb, double lambda = 0.001, bool svd = false) {
  MatrixXd jacb_inv(jacb.rows(), jacb.cols());

  if (svd) {
    // Todo: Do we need to check if jacb is a square matrix?
    jacb_inv.setZero();
    Eigen::JacobiSVD<math::MatrixXd> svd(
        jacb, ::Eigen::ComputeFullU | ::Eigen::ComputeFullV);

    const double wMin = svd.singularValues().minCoeff();
    const double tol  = 1e-9;
    double lambdaSqr
        = wMin < tol ? (1 - ::std::pow((wMin / tol), 2)) * ::std::pow(lambda, 2)
                     : 0;

    for (::std::ptrdiff_t i = 0; i < svd.nonzeroSingularValues(); ++i) {
      jacb_inv.noalias()
          += (svd.singularValues()(i)
              / (::std::pow(svd.singularValues()(i), 2) + lambdaSqr)
              * svd.matrixV().col(i) * svd.matrixU().col(i).transpose());
    }
  }
  else {
    math::MatrixXd damping
        = std::pow(lambda, 2)
          * math::MatrixXd::Identity(jacb.rows(), jacb.rows());
    jacb_inv = jacb.transpose() * (jacb * jacb.transpose() + damping).inverse();
  }

  // J.completeOrthogonalDecomposition().pseudoInverse();

  return jacb_inv;
}

#include "crobot/math/detail/matrix_impl.hpp"

} // namespace crobot::math
