#ifndef OBJECT_REORIENTATION_SOLVER_HPP
#define OBJECT_REORIENTATION_SOLVER_HPP

#include <limits>
#include <memory>

#include "crobot/common/common.hpp"
#include "crobot/geometry/mesh_patches.hpp"
#include "crobot/math/math.hpp"

namespace crobot {

/**
 * @brief Standalone solver for hand/object re-orientation (Eq. 43).
 *
 * This class is intentionally independent from EquilibriumGrasp. It solves
 * for the orientation increment delta_psi and contact forces that satisfy
 * force balance, torque balance, and polyhedral friction constraints.
 */
class ObjectReorientationSolver {
public:
  using Ptr      = std::shared_ptr<ObjectReorientationSolver>;
  using ConstPtr = std::shared_ptr<const ObjectReorientationSolver>;

  /**
   * @brief Result of the hand/object re-orientation solve.
   */
  struct Result
  {
    /// True when the NLP converged to a feasible adjustment.
    bool feasible = false;

    /// IPOPT termination code (IpoptSolver::Status values).
    int solver_status = -1;

    /// Optimal objective value ||delta_psi||^2.
    double objective = std::numeric_limits<double>::infinity();

    /// Optimal orientation adjustment in body coordinates (axis-angle vector).
    math::Vector3d delta_psi = math::Vector3d::Zero();

    /// Updated z-axis direction in body coordinates after applying delta_psi.
    math::Vector3d adjusted_z = math::Vector3d::Zero();

    /// Contact forces ordered to match the input vertex index list.
    std::vector<math::Vector3d> forces;
  };

  ObjectReorientationSolver(
      geometry::MeshPatches::ConstPtr mesh, double mu, int num_generators = 8);

  ~ObjectReorientationSolver();

  /**
   * @brief Prepare and cache the optimization problem structure for contacts.
   *
   * Call once when the contact set is fixed, then call solve(...) repeatedly
   * without vertex indices to avoid reconstruction overhead.
   */
  void configure_contacts(const std::vector<int>& vertex_indices) const;

  /**
   * @brief Solve using the cached contact configuration.
   *
   * Requires configure_contacts(...) to have been called beforehand.
   */
  Result solve(
      const math::Vector3d& z_axis_in_body,
      double gravity_force_magnitude,
      const std::vector<math::Vector3d>& initial_forces = {},
      bool verbose                                      = false) const;

  /**
   * @brief Solve the hand/object re-orientation problem.
   */
  Result solve(
      const std::vector<int>& vertex_indices,
      const math::Vector3d& z_axis_in_body,
      double gravity_force_magnitude,
      const std::vector<math::Vector3d>& initial_forces = {},
      bool verbose                                      = false) const;

private:
  struct Impl;

  void ensure_impl() const;

  geometry::MeshPatches::ConstPtr m_mesh;
  double m_mu;
  int m_num_generators;
  mutable std::unique_ptr<Impl> m_impl;
};

} // namespace crobot

#endif // OBJECT_REORIENTATION_SOLVER_HPP
