#include <algorithm>
#include <stdexcept>
#include <tuple>

#include "monotone_internal.hpp"

namespace crobot::patch_tree {
MonotoneDecomposition decompose_monotone(
    const open3d::geometry::TriangleMesh& mesh,
    const MonotoneOptions& options,
    const std::vector<int>& nonmanifold_vertices) {
  auto positive
      = [](double value) { return std::isfinite(value) && value > 0; };
  auto fraction = [&](double value) { return positive(value) && value <= 1; };
  for (double r : options.radii)
    if (!positive(r))
      throw std::invalid_argument("Monotone radii must be finite and positive");
  if (options.normal_smoothing_iterations < 0
      || !positive(options.preferred_radius)
      || !positive(options.max_curvature_error)
      || !positive(options.rms_curvature_error)
      || !fraction(options.max_uncertain_fraction)
      || !fraction(options.tiny_area_fraction)
      || !positive(options.relief_limit) || !positive(options.diameter_limit)
      || !positive(options.normal_angle_degrees)
      || options.normal_angle_degrees > 180
      || !positive(options.raw_crease_degrees)
      || options.raw_crease_degrees > 180
      || !fraction(options.parent_detail_fraction)
      || !fraction(options.global_detail_fraction)
      || !fraction(options.persistent_fraction)
      || !fraction(options.persistent_limit_factor)
      || !positive(options.minimum_recipient_ratio)
      || !fraction(options.ambiguous_boundary_share))
    throw std::invalid_argument("Invalid monotone decomposition options");
  const auto g = detail::geometry(
      mesh, nonmanifold_vertices, options.normal_smoothing_iterations);
  MonotoneDecomposition result;
  result.diameter         = g.diameter;
  result.components       = g.components;
  result.analysis_normals = g.normals;
  if (g.faces.empty())
    return result;
  std::array<std::array<detail::Evidence, 2>, 3> evidence;
  std::array<std::array<MonotoneCut, 2>, 3> cuts;
  std::tuple<bool, double, bool, double> best{
      true, std::numeric_limits<double>::infinity(), true, 1};
  int selected_radius = 0, selected_method = 0;
  for (int r = 0; r < 3; r++)
    for (int m = 0; m < 2; m++) {
      evidence[r][m] = detail::estimate(
          g, options.radii[r], static_cast<CurvatureEstimator>(m), options);
      cuts[r][m]     = detail::merge_regions(g, evidence[r][m], options);
      double unknown = 0;
      bool quality   = true;
      for (const auto& region : cuts[r][m].regions) {
        unknown += region.area_fraction * region.uncertain_fraction;
        quality = quality
                  && region.max_curvature_error
                         <= options.max_curvature_error + 1e-10
                  && region.rms_curvature_error
                         <= options.rms_curvature_error + 1e-10;
      }
      quality    = quality && unknown <= options.max_uncertain_fraction;
      auto score = std::make_tuple(
          !quality,
          std::abs(std::log2(options.radii[r] / options.preferred_radius)),
          m == 0,
          unknown);
      if (score < best) {
        best            = score;
        selected_radius = r;
        selected_method = m;
      }
    }
  const auto& selected = evidence[selected_radius][selected_method];
  result.estimator     = static_cast<CurvatureEstimator>(selected_method);
  result.radius        = options.radii[selected_radius];
  result.core_field_limits_met = !std::get<0>(best);
  result.cuts[0] = std::move(cuts[selected_radius][selected_method]);
  std::vector<const detail::Evidence*> scales;
  for (int r = 0; r < 3; r++)
    if (r != selected_radius)
      scales.push_back(&evidence[r][selected_method]);
  result.cuts[1] = detail::group_details(
      g, selected, result.cuts[0], scales, options, result.absorbed_detail);
  result.cuts[2]
      = detail::assign_uncertain(g, selected, result.cuts[1], options, result);
  for (size_t f = 0; f < g.faces.size(); f++) {
    const auto& v = selected.values[f];
    result.gaussian_curvature.push_back(v[0]);
    result.mean_curvature.push_back(v[1]);
    result.principal_curvatures.push_back({v[2], v[3]});
    result.uncertain.push_back(selected.invalid[f]);
  }
  return result;
}
} // namespace crobot::patch_tree
