#pragma once

#include "crobot/geometry/icp.hpp"
#include "crobot/math/surface_patch.hpp"

namespace crobot::geometry {

struct SurfaceICPOptions
{
  ICPOptions icp;
  int projection_grid_size      = 3;
  double projection_tolerance   = 1e-8;
  int projection_max_iterations = 100;
};

// Registers measured points to their assigned parametric patches. The returned
// transform is a common world-frame correction to the original patch frames:
// new_patch_frame = result.transform * original_patch_frame. Patches are copied
// and never mutated. Correspondences always run measurements -> patches;
// options.icp.direction is not used. aligned_points contains final projections.
class SurfaceICPSolver {
public:
  SurfaceICPSolver(
      const std::vector<math::SurfacePatch::ConstPtr>& patches,
      const std::vector<ICPPoints>& measurements);
  ICPResult solve(const SurfaceICPOptions& options = {}) const;

private:
  std::vector<math::SurfacePatch> m_patches;
  std::vector<ICPPoints> m_measurements;
};

} // namespace crobot::geometry
