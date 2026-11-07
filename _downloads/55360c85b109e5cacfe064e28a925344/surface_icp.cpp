#include "crobot/geometry/surface_icp.hpp"

#include <cmath>
#include <stdexcept>

#include "crobot/geometry/detail/icp_internal.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/ipopt_solver.hpp"
#include "crobot/optimization/problem.hpp"
#include "crobot/optimization/variable_set.hpp"

namespace crobot::geometry {
namespace {

class SurfaceParameters : public optimization::VariableSet {
public:
  explicit SurfaceParameters(const math::SurfacePatch& patch)
    : VariableSet(2, "uv"),
      m_value((patch.get_lb() + patch.get_ub()) / 2),
      m_bounds{
          {patch.get_lb()[0], patch.get_ub()[0]},
          {patch.get_lb()[1], patch.get_ub()[1]}} {
  }
  void set_variables(const math::VectorXd& value) override {
    m_value = value;
  }
  math::VectorXd get_values() const override {
    return m_value;
  }
  optimization::VecBound get_bounds() const override {
    return m_bounds;
  }

private:
  math::Vector2d m_value;
  optimization::VecBound m_bounds;
};

class SurfaceDistance : public optimization::CostTerm {
public:
  explicit SurfaceDistance(std::shared_ptr<math::SurfacePatch> patch)
    : CostTerm("distance", {"uv"}), surface(std::move(patch)) {
  }
  double get_cost(const math::VectorXd& x) const override {
    return (surface->evaluate(x) - point).squaredNorm();
  }
  void fill_jacobian_block(
      optimization::Component::Ptr variables,
      math::SparseJacobian& jacobian) const override {
    const math::Vector2d uv          = variables->get_values();
    const math::Vector3d difference  = surface->evaluate(uv) - point;
    const math::Matrix32 derivatives = surface->jacb(uv);
    for (int i = 0; i < 2; ++i)
      jacobian.coeffRef(0, i) = 2 * difference.dot(derivatives.col(i));
  }
  math::Vector3d point = math::Vector3d::Zero();
  std::shared_ptr<math::SurfacePatch> surface;
};

struct ProjectionProblem
{
  explicit ProjectionProblem(const math::SurfacePatch& patch)
    : surface(std::make_shared<math::SurfacePatch>(patch)),
      parameters(std::make_shared<SurfaceParameters>(patch)),
      cost(std::make_shared<SurfaceDistance>(surface)) {
    problem.add_variable_set(parameters);
    problem.add_cost_set(cost);
  }
  std::shared_ptr<math::SurfacePatch> surface;
  std::shared_ptr<SurfaceParameters> parameters;
  std::shared_ptr<SurfaceDistance> cost;
  optimization::Problem problem;
  std::vector<math::Vector2d> previous_parameters;
};

} // namespace

SurfaceICPSolver::SurfaceICPSolver(
    const std::vector<math::SurfacePatch::ConstPtr>& patches,
    const std::vector<ICPPoints>& measurements)
  : m_measurements(measurements) {
  if (patches.empty() || patches.size() != measurements.size())
    throw std::invalid_argument(
        "Surface ICP requires matching patches and measurement groups");
  size_t count = 0;
  for (size_t i = 0; i < patches.size(); ++i) {
    if (!patches[i])
      throw std::invalid_argument("Surface ICP patch is null");
    detail::validate_transform(patches[i]->get_tform());
    if (!patches[i]->get_lb().allFinite() || !patches[i]->get_ub().allFinite()
        || (patches[i]->get_lb().array() >= patches[i]->get_ub().array()).any())
      throw std::invalid_argument(
          "Surface ICP requires finite, increasing parameter bounds");
    if (!measurements[i].empty())
      detail::validate_points(measurements[i]);
    count += measurements[i].size();
    m_patches.push_back(*patches[i]);
  }
  if (count < 3)
    throw std::invalid_argument(
        "Surface ICP needs at least three measurements");
}

ICPResult SurfaceICPSolver::solve(const SurfaceICPOptions& options) const {
  detail::validate_options(options.icp);
  if (options.projection_grid_size < 2 || options.projection_grid_size > 32
      || !std::isfinite(options.projection_tolerance)
      || options.projection_tolerance <= 0
      || options.projection_max_iterations <= 0)
    throw std::invalid_argument("Invalid surface projection options");
  optimization::IpoptSolver solver;
  solver.set_option("print_level", 0);
  solver.set_option("mu_strategy", "adaptive");
  solver.set_option("tol", options.projection_tolerance);
  solver.set_option("max_iter", options.projection_max_iterations);
  solver.set_option("max_cpu_time", 1.0);
  std::vector<std::unique_ptr<ProjectionProblem>> projections;
  for (const auto& patch : m_patches)
    projections.push_back(std::make_unique<ProjectionProblem>(patch));
  const double max_squared_distance = options.icp.max_correspondence_distance
                                      * options.icp.max_correspondence_distance;
  auto result = detail::run_icp(
      math::Matrix4d::Identity(),
      options.icp,
      [&](const math::Matrix4d& transform,
          ICPPoints& source,
          ICPPoints& target) {
        for (size_t i = 0; i < m_patches.size(); ++i) {
          auto& projection           = *projections[i];
          const math::Matrix4d frame = transform * m_patches[i].get_tform();
          projection.surface->set_rotm(frame.block<3, 3>(0, 0));
          projection.surface->set_bpos(frame.block<3, 1>(0, 3));
          std::vector<math::Vector2d> seeds;
          const auto lower = projection.surface->get_lb();
          const auto upper = projection.surface->get_ub();
          for (int u = 0; u < options.projection_grid_size; ++u)
            for (int v = 0; v < options.projection_grid_size; ++v) {
              const math::Vector2d fraction(
                  (u + 0.5) / options.projection_grid_size,
                  (v + 0.5) / options.projection_grid_size);
              seeds.push_back(lower + fraction.cwiseProduct(upper - lower));
            }
          for (size_t j = 0; j < m_measurements[i].size(); ++j) {
            projection.cost->point = m_measurements[i][j];
            auto candidates        = seeds;
            if (j < projection.previous_parameters.size())
              candidates.push_back(projection.previous_parameters[j]);
            double best_cost          = std::numeric_limits<double>::infinity();
            math::Vector2d best_uv    = (lower + upper) / 2;
            math::Vector3d best_point = math::Vector3d::Zero();
            for (const auto& seed : candidates) {
              projection.parameters->set_variables(seed);
              try {
                solver.solve(projection.problem);
              }
              catch (const std::exception&) {
                continue;
              }
              if (solver.get_return_status()
                  != optimization::IpoptSolver::SUCCEEDED)
                continue;
              // IPOPT may relax bounds slightly; evaluate within the true
              // domain.
              const math::Vector2d uv = projection.parameters->get_values()
                                            .cwiseMax(lower)
                                            .cwiseMin(upper);
              const math::Vector3d point = projection.surface->evaluate(uv);
              const double cost
                  = (point - projection.cost->point).squaredNorm();
              if (point.allFinite() && std::isfinite(cost)
                  && cost < best_cost) {
                best_cost  = cost;
                best_uv    = uv;
                best_point = point;
              }
            }
            if (!std::isfinite(best_cost))
              return ICPStatus::ProjectionFailed;
            if (j == projection.previous_parameters.size())
              projection.previous_parameters.push_back(best_uv);
            else
              projection.previous_parameters[j] = best_uv;
            if (best_cost > max_squared_distance)
              continue;
            source.push_back(best_point);
            target.push_back(projection.cost->point);
          }
        }
        return ICPStatus::Converged;
      });
  result.aligned_points = result.source_correspondences;
  return result;
}

} // namespace crobot::geometry
