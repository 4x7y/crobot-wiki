#ifndef EQUILIBRIUM_GRASP_HPP
#define EQUILIBRIUM_GRASP_HPP

#include <limits>

#include "crobot/common/common.hpp"
#include "crobot/geometry/mesh_patches.hpp"
#include "crobot/math/math.hpp"

namespace crobot {

/**
 * @brief Solve the equilibrium-grasp force allocation problem for fixed
 * contact vertices.
 *
 * For fixed contact positions r_i, inward normals n_i, and a target gravity
 * direction u, this class solves the equivalent max-lambda formulation
 *
 *   max_{f_1,...,f_k, lambda} lambda
 *
 * subject to
 *
 *   sum_i f_i = lambda u,
 *   sum_i r_i x f_i = 0,
 *   sum_i f_i dot n_i = 1,
 *   f_i dot n_i >= 0,
 *   f_i dot f_i <= (1 + mu^2) (f_i dot n_i)^2,   for 1 <= i <= k.
 *
 * The first two constraints impose force and torque equilibrium and the third
 * normalizes the total compressive effort. The friction cone is linearized with
 * a K-sided polyhedral pyramid, turning the problem into a linear program (LP)
 * solved with the COIN-OR CLP dual-simplex solver. The returned quality is the
 * optimal value of lambda; a larger lambda means the grasp can generate a
 * larger resultant force along u under unit total normal force.
 *
 * Because the polyhedral cone is an inner approximation of the true Coulomb
 * cone, the LP result is a lower bound on the true quality. More generators K
 * give a tighter approximation.
 */
class EquilibriumGrasp {
public:
  using Ptr      = std::shared_ptr<EquilibriumGrasp>;
  using ConstPtr = std::shared_ptr<const EquilibriumGrasp>;

  /**
   * @brief Result of a single equilibrium-grasp solve.
   */
  struct Result
  {
    /// True when the solve produced a positive-quality equilibrium solution.
    bool feasible = false;

    /// Optimal value of the max-lambda formulation.
    double lambda = -std::numeric_limits<double>::infinity();

    /// Solver-specific termination code (ClpSolver::Status values).
    int solver_status = -1;

    /// Contact forces ordered to match the input vertex index list.
    std::vector<math::Vector3d> forces;
  };

  /**
   * @brief Result of the NLP grasp quality solve (problem 34).
   */
  struct NlpResult
  {
    /// True when the NLP produced a positive-quality solution.
    bool feasible = false;

    /// Optimal quality value (c(u)^T g at the solution).
    double quality = 0.0;

    /// Optimal gravity direction (unit vector).
    math::Vector3d u = math::Vector3d::Zero();
  };

  /**
   * @brief Construct an equilibrium solver for a mesh and friction
   * coefficient.
   *
   * @param mesh            Mesh patch representation that provides contact
   *                        positions and inward normals.
   * @param mu              Coulomb friction coefficient used at every contact.
   * @param num_generators  Number of sides K of the polyhedral friction cone
   *                        approximation. Higher values improve accuracy at
   *                        the cost of a larger LP. Default 8 is a good
   *                        balance; use 4 for speed or 16 for high fidelity.
   */
  EquilibriumGrasp(
      geometry::MeshPatches::ConstPtr mesh, double mu, int num_generators = 8);

  /**
   * @brief Solve the equilibrium problem for gravity aligned with +z.
   *
   * This is a convenience overload for solve(vertex_indices, UnitZ()).
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param verbose If true, enables IPOPT solver output.
   * @return Solve result including feasibility, quality, and contact forces.
   */
  Result solve(
      const std::vector<int>& vertex_indices, bool verbose = false) const;

  /**
   * @brief Solve the equilibrium problem for a specified gravity direction.
   *
   * The problem maximizes lambda subject to:
   * - summed contact force equals lambda times @p gravity_dir
   * - summed contact torque is zero
   * - summed normal force equals one
   * - each contact force remains compressive and inside its friction cone
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param gravity_dir Desired resultant force direction in the body/world
   * frame used by the formulation.
   * @param verbose If true, enables IPOPT solver output.
   * @return Solve result including feasibility, quality, and contact forces.
   */
  Result solve(
      const std::vector<int>& vertex_indices,
      const math::Vector3d& gravity_dir,
      bool verbose = false) const;

  /**
   * @brief Test equilibrium feasibility for gravity aligned with +z.
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param lambda_tol Minimum acceptable lambda to treat the grasp as
   * feasible.
   * @param verbose If true, enables IPOPT solver output.
   * @return True if a feasible solution with lambda greater than
   * @p lambda_tol is found.
   */
  bool is_equilibrium_grasp(
      const std::vector<int>& vertex_indices,
      double lambda_tol = 1e-6,
      bool verbose      = false) const;

  /**
   * @brief Test equilibrium feasibility for a specified gravity direction.
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param gravity_dir Desired resultant force direction in the formulation.
   * @param lambda_tol Minimum acceptable lambda to treat the grasp as
   * feasible.
   * @param verbose If true, enables IPOPT solver output.
   * @return True if a feasible solution with lambda greater than
   * @p lambda_tol is found.
   */
  bool is_equilibrium_grasp(
      const std::vector<int>& vertex_indices,
      const math::Vector3d& gravity_dir,
      double lambda_tol = 1e-6,
      bool verbose      = false) const;

  /**
   * @brief Compute equilibrium quality for gravity aligned with +z.
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param verbose If true, enables IPOPT solver output.
   * @return Optimal lambda value. Non-positive or non-finite values indicate
   * failure or infeasibility.
   */
  double get_quality(
      const std::vector<int>& vertex_indices, bool verbose = false) const;

  /**
   * @brief Compute equilibrium quality for a specified gravity direction.
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param gravity_dir Desired resultant force direction in the formulation.
   * @param verbose If true, enables IPOPT solver output.
   * @return Optimal lambda value. Non-positive or non-finite values indicate
   * failure or infeasibility.
   */
  double get_quality(
      const std::vector<int>& vertex_indices,
      const math::Vector3d& gravity_dir,
      bool verbose = false) const;

  /**
   * @brief Test strong equilibrium for a cone of gravity directions (Eq. 14).
   *
   * Models gravity direction uncertainty as a cone in the object body frame
   * with central axis @p cone_axis and half-angle @p cone_half_angle. The
   * cross-section of this cone at unit height forms a disk D. Its bounding
   * circle is approximated by a regular s-gon with vertices gamma_j.
   *
   * For each gamma_j the following feasibility LP is solved (Eq. 14):
   *
   *   find  alpha_{ik} >= 0
   *   s.t.  sum_ik alpha_{ik} * g_{ik}          = gamma_j   (force balance)
   *         sum_ik alpha_{ik} * (r_i x g_{ik})  = 0         (torque balance)
   *
   * The grasp attains strong equilibrium iff the LP is feasible for all j.
   *
   * @param vertex_indices   Contact vertex ids on the mesh.
   * @param cone_axis        Central gravity direction (need not be normalized).
   * @param cone_half_angle  Half-angle in radians. Must be in [0, pi/2).
   *                         Pass 0 to test a single direction (equivalent to
   *                         is_equilibrium_grasp for that direction).
   * @param num_disk_samples Number of s-gon vertices (the s in Eq. 14).
   * @param verbose          If true, CLP prints status for each LP.
   * @return True iff all @p num_disk_samples directions are feasible.
   */
  bool is_strong_equilibrium_grasp(
      const std::vector<int>& vertex_indices,
      const math::Vector3d& cone_axis,
      double cone_half_angle,
      int num_disk_samples = 8,
      bool verbose         = false) const;

  /**
   * @brief Solve the NLP grasp quality problem (34) using IPOPT.
   *
   * Maximizes c(u)^T g over the gravity direction u in the cone defined by
   * @p cone_axis and @p cone_half_angle, and the dual variable vector g,
   * subject to the dual feasibility constraint Ag <= b, g >= 0, and the
   * sphere/cone constraints on u.
   *
   * This formulates the inner LP duality result from the paper as a single
   * NLP and solves it with IPOPT, avoiding the linearization of the friction
   * cone needed for a pure LP approach.
   *
   * @param vertex_indices   Contact vertex ids on the mesh.
   * @param cone_axis        Central gravity direction (need not be normalized).
   * @param cone_half_angle  Half-angle of the gravity uncertainty cone (rad).
   * @param verbose          If true, enables IPOPT output.
   * @return NlpResult with the optimal quality, gravity direction, and
   * feasibility flag.
   */
  NlpResult solve_quality_nlp(
      const std::vector<int>& vertex_indices,
      const math::Vector3d& cone_axis,
      double cone_half_angle,
      bool verbose = false) const;

  /**
   * @brief Solve the NLP quality problem for a fixed gravity direction (no
   * cone uncertainty).
   *
   * Equivalent to solve_quality_nlp with cone_axis = UnitZ and
   * cone_half_angle = 0.
   *
   * @param vertex_indices Contact vertex ids on the mesh.
   * @param verbose        If true, enables IPOPT output.
   * @return NlpResult with the optimal quality.
   */
  NlpResult solve_quality_nlp(
      const std::vector<int>& vertex_indices, bool verbose = false) const;

private:
  geometry::MeshPatches::ConstPtr m_mesh;
  double m_mu;
  int m_num_generators;
};

} // namespace crobot

#endif // EQUILIBRIUM_GRASP_HPP