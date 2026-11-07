#include "crobot/geometry/icp.hpp"

#include <cmath>
#include <stdexcept>

#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/PointCloud.h>

#include "crobot/geometry/detail/icp_internal.hpp"
#include "crobot/math/quaternion.hpp"

namespace crobot::geometry {
namespace {

math::Matrix4d quaternion_matrix(const math::Vector3d& p, bool left) {
  math::Matrix4d matrix    = math::Matrix4d::Zero();
  matrix.block<3, 1>(1, 0) = p;
  matrix.block<1, 3>(0, 1) = -p.transpose();
  math::Matrix3d cross;
  cross << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
  matrix.block<3, 3>(1, 1) = left ? cross : -cross;
  return matrix;
}

ICPStatus fit_rigid(
    const ICPPoints& source,
    const ICPPoints& target,
    math::Matrix4d& transform) {
  if (source.size() < 3 || source.size() != target.size())
    return ICPStatus::InsufficientCorrespondences;
  math::Vector3d source_center = math::Vector3d::Zero();
  math::Vector3d target_center = math::Vector3d::Zero();
  for (size_t i = 0; i < source.size(); ++i) {
    source_center += source[i];
    target_center += target[i];
  }
  source_center /= source.size();
  target_center /= target.size();
  math::Matrix4d matrix = math::Matrix4d::Zero();
  for (size_t i = 0; i < source.size(); ++i) {
    matrix += quaternion_matrix(source[i] - source_center, false).transpose()
              * quaternion_matrix(target[i] - target_center, true);
  }
  if (!matrix.allFinite())
    return ICPStatus::NumericalFailure;
  Eigen::SelfAdjointEigenSolver<math::Matrix4d> eigen(matrix);
  if (eigen.info() != Eigen::Success)
    return ICPStatus::NumericalFailure;
  const auto values  = eigen.eigenvalues();
  const double scale = values.cwiseAbs().maxCoeff();
  // A repeated largest eigenvalue means the rotation is not identifiable
  // (e.g. coincident or collinear correspondences). Planar data is allowed.
  if (scale == 0 || values[3] - values[2] <= 1e-12 * scale)
    return ICPStatus::DegenerateCorrespondences;
  const math::Vector4d vector = eigen.eigenvectors().col(3);
  const math::Quaternion quaternion(vector[0], vector[1], vector[2], vector[3]);
  transform                   = math::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
  transform.block<3, 1>(0, 3)
      = target_center - transform.block<3, 3>(0, 0) * source_center;
  return transform.allFinite() ? ICPStatus::Converged
                               : ICPStatus::NumericalFailure;
}

} // namespace

const char* to_string(ICPStatus status) {
  switch (status) {
    case ICPStatus::Converged:
      return "converged";
    case ICPStatus::MaxIterations:
      return "maximum iterations reached";
    case ICPStatus::InsufficientCorrespondences:
      return "insufficient correspondences";
    case ICPStatus::DegenerateCorrespondences:
      return "degenerate correspondences";
    case ICPStatus::ProjectionFailed:
      return "surface projection failed";
    case ICPStatus::NumericalFailure:
      return "numerical failure";
  }
  return "unknown status";
}

namespace detail {

void validate_points(const ICPPoints& points) {
  if (points.empty())
    throw std::invalid_argument("ICP point clouds must not be empty");
  for (const auto& point : points)
    if (!point.allFinite())
      throw std::invalid_argument("ICP points must be finite");
}

void validate_transform(const math::Matrix4d& transform) {
  const math::Matrix3d rotation = transform.block<3, 3>(0, 0);
  if (!transform.allFinite()
      || !transform.row(3).isApprox(
          math::Vector4d(0, 0, 0, 1).transpose(), 1e-8)
      || !(rotation.transpose() * rotation)
              .isApprox(math::Matrix3d::Identity(), 1e-8)
      || std::abs(rotation.determinant() - 1) > 1e-8)
    throw std::invalid_argument(
        "ICP initial transform must be rigid and finite");
}

void validate_options(const ICPOptions& options) {
  if (options.max_iterations <= 0
      || !std::isfinite(options.translation_tolerance)
      || options.translation_tolerance < 0
      || !std::isfinite(options.rotation_tolerance)
      || options.rotation_tolerance < 0
      || !std::isfinite(options.rmse_tolerance) || options.rmse_tolerance < 0
      || std::isnan(options.max_correspondence_distance)
      || options.max_correspondence_distance <= 0
      || (options.direction != ICPDirection::SourceToTarget
          && options.direction != ICPDirection::TargetToSource))
    throw std::invalid_argument("Invalid ICP options");
}

ICPPoints transform_points(
    const ICPPoints& points, const math::Matrix4d& transform) {
  ICPPoints result;
  result.reserve(points.size());
  for (const auto& point : points)
    result.push_back(
        transform.block<3, 3>(0, 0) * point + transform.block<3, 1>(0, 3));
  return result;
}

ICPResult run_icp(
    const math::Matrix4d& initial_transform,
    const ICPOptions& options,
    const Correspondences& correspondences) {
  validate_options(options);
  validate_transform(initial_transform);
  ICPResult result;
  result.transform     = initial_transform;
  double previous_rmse = std::numeric_limits<double>::infinity();
  bool small_step      = false;
  for (;;) {
    result.source_correspondences.clear();
    result.target_correspondences.clear();
    result.rmse   = std::numeric_limits<double>::infinity();
    result.status = correspondences(
        result.transform,
        result.source_correspondences,
        result.target_correspondences);
    if (result.status != ICPStatus::Converged)
      return result;
    math::Matrix4d increment;
    result.status = fit_rigid(
        result.source_correspondences,
        result.target_correspondences,
        increment);
    if (result.status != ICPStatus::Converged)
      return result;
    double squared_error = 0;
    for (size_t i = 0; i < result.source_correspondences.size(); ++i)
      squared_error += (result.source_correspondences[i]
                        - result.target_correspondences[i])
                           .squaredNorm();
    result.rmse
        = std::sqrt(squared_error / result.source_correspondences.size());
    if (!std::isfinite(result.rmse)) {
      result.status = ICPStatus::NumericalFailure;
      return result;
    }
    // Assess freshly recomputed correspondences at the returned transform.
    if (result.iterations > 0
        && (small_step
            || std::abs(previous_rmse - result.rmse)
                   <= options.rmse_tolerance)) {
      result.status = ICPStatus::Converged;
      return result;
    }
    if (result.iterations == options.max_iterations) {
      result.status = ICPStatus::MaxIterations;
      return result;
    }
    small_step
        = increment.block<3, 1>(0, 3).norm() <= options.translation_tolerance
          && Eigen::AngleAxisd(math::Matrix3d(increment.block<3, 3>(0, 0)))
                     .angle()
                 <= options.rotation_tolerance;
    previous_rmse    = result.rmse;
    result.transform = (increment * result.transform).eval();
    if (!result.transform.allFinite()) {
      result.status = ICPStatus::NumericalFailure;
      return result;
    }
    ++result.iterations;
  }
}

} // namespace detail

ICPSolver::ICPSolver(
    const ICPPoints& source,
    const ICPPoints& target,
    const math::Matrix4d& initial_transform)
  : m_source(source), m_target(target), m_initial_transform(initial_transform) {
  detail::validate_points(source);
  detail::validate_points(target);
  detail::validate_transform(initial_transform);
}

math::Matrix4d ICPSolver::solve_quat(
    const ICPPoints& source, const ICPPoints& target) {
  detail::validate_points(source);
  detail::validate_points(target);
  math::Matrix4d result;
  const auto status = fit_rigid(source, target, result);
  if (status != ICPStatus::Converged)
    throw std::invalid_argument(to_string(status));
  return result;
}

ICPResult ICPSolver::solve(const ICPOptions& options) const {
  detail::validate_options(options);
  const bool forward = options.direction == ICPDirection::SourceToTarget;
  open3d::geometry::PointCloud cloud;
  cloud.points_ = forward ? m_target : m_source;
  const open3d::geometry::KDTreeFlann tree(cloud);
  const double max_squared_distance = options.max_correspondence_distance
                                      * options.max_correspondence_distance;
  auto result = detail::run_icp(
      m_initial_transform,
      options,
      [&](const math::Matrix4d& transform,
          ICPPoints& source,
          ICPPoints& target) {
        const auto aligned = detail::transform_points(m_source, transform);
        const math::Matrix3d rotation    = transform.block<3, 3>(0, 0);
        const math::Vector3d translation = transform.block<3, 1>(0, 3);
        std::vector<int> indices;
        std::vector<double> distances;
        const size_t count = forward ? aligned.size() : m_target.size();
        for (size_t i = 0; i < count; ++i) {
          // In reverse mode query in source coordinates, so the tree is fixed.
          const math::Vector3d query
              = forward ? aligned[i]
                        : math::Vector3d(
                            rotation.transpose() * (m_target[i] - translation));
          if (!query.allFinite())
            return ICPStatus::NumericalFailure;
          if (tree.SearchKNN(query, 1, indices, distances) != 1)
            return ICPStatus::NumericalFailure;
          if (distances[0] > max_squared_distance)
            continue;
          source.push_back(forward ? aligned[i] : aligned[indices[0]]);
          target.push_back(forward ? m_target[indices[0]] : m_target[i]);
        }
        return ICPStatus::Converged;
      });
  result.aligned_points = detail::transform_points(m_source, result.transform);
  return result;
}

} // namespace crobot::geometry
