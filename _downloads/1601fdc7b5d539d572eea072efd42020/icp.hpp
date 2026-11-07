#pragma once

#include <limits>
#include <vector>

#include "crobot/math/matrix.hpp"

namespace crobot::geometry {

using ICPPoints = std::vector<math::Vector3d>;

enum class ICPDirection
{
  SourceToTarget,
  TargetToSource
};
enum class ICPStatus
{
  Converged,
  MaxIterations,
  InsufficientCorrespondences,
  DegenerateCorrespondences,
  ProjectionFailed,
  NumericalFailure
};

struct ICPOptions
{
  int max_iterations           = 100;
  double translation_tolerance = 1e-6;
  double rotation_tolerance    = 1e-6; // radians
  double rmse_tolerance = 1e-8; // change between iterations, in point units
  double max_correspondence_distance = std::numeric_limits<double>::infinity();
  ICPDirection direction             = ICPDirection::SourceToTarget;
};

struct ICPResult
{
  // Maps the original source coordinates into target coordinates, including
  // the initial transform. Convergence is local, not a guarantee of a good fit.
  math::Matrix4d transform = math::Matrix4d::Identity();
  ICPStatus status         = ICPStatus::MaxIterations;
  int iterations           = 0; // number of rigid updates
  double rmse              = std::numeric_limits<double>::infinity();
  ICPPoints aligned_points;
  ICPPoints source_correspondences;
  ICPPoints target_correspondences;

  bool converged() const {
    return status == ICPStatus::Converged;
  }
};

const char* to_string(ICPStatus status);

// Numerical registration only: no simulation, visualization, or input mutation.
// Invalid inputs/options throw std::invalid_argument; solve failures are
// reported in ICPResult. Point clouds need not have equal sizes.
class ICPSolver {
public:
  ICPSolver(
      const ICPPoints& source,
      const ICPPoints& target,
      const math::Matrix4d& initial_transform = math::Matrix4d::Identity());

  ICPResult solve(const ICPOptions& options = {}) const;

  // Paired, finite, equally sized arrays of at least three points are required.
  // Throws std::invalid_argument for ambiguous/degenerate rigid fits.
  static math::Matrix4d solve_quat(
      const ICPPoints& source, const ICPPoints& target);

private:
  ICPPoints m_source;
  ICPPoints m_target;
  math::Matrix4d m_initial_transform;
};

} // namespace crobot::geometry
