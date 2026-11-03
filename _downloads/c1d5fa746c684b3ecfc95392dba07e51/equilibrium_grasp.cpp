#include <cmath>

#include "crobot/grasping/equilibrium_grasp.hpp"
#include "crobot/optimization/bounds.hpp"
#include "crobot/optimization/clp_solver.hpp"
#include "crobot/optimization/constraint_set.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/ipopt_solver.hpp"
#include "crobot/optimization/linear_program.hpp"
#include "crobot/optimization/problem.hpp"
#include "crobot/optimization/variable_set.hpp"

using namespace crobot;

namespace {

// Returns an orthonormal tangent frame (t1, t2) for the given unit normal.
std::pair<math::Vector3d, math::Vector3d> compute_tangent_frame(
    const math::Vector3d& n) {
  math::Vector3d t1;
  if (std::abs(n.dot(math::Vector3d::UnitX())) < 0.9) {
    t1 = n.cross(math::Vector3d::UnitX()).normalized();
  }
  else {
    t1 = n.cross(math::Vector3d::UnitY()).normalized();
  }
  return {t1, n.cross(t1).normalized()};
}

// Returns K edge generators of the polyhedral friction cone approximation.
// Each generator g_k = n + mu*(cos(theta_k)*t1 + sin(theta_k)*t2) lies on
// the boundary of the linearized cone, with theta_k = 2*pi*k/K.
std::vector<math::Vector3d> compute_generators(
    const math::Vector3d& n,
    const math::Vector3d& t1,
    const math::Vector3d& t2,
    double mu,
    int K) {
  static constexpr double two_pi = 2.0 * 3.14159265358979323846;
  std::vector<math::Vector3d> gens;
  gens.reserve(K);
  for (int k = 0; k < K; ++k) {
    double angle = two_pi * k / K;
    gens.push_back(n + mu * (std::cos(angle) * t1 + std::sin(angle) * t2));
  }
  return gens;
}

// Contact positions and per-contact cone generators for a set of vertices.
struct ContactGeometry
{
  std::vector<math::Vector3d> positions;
  std::vector<std::vector<math::Vector3d>> generators; // [contact_i][cone_k]
};

ContactGeometry build_contact_geometry(
    const geometry::MeshPatches& mesh,
    const std::vector<int>& vertex_indices,
    double mu,
    int K) {
  ContactGeometry geom;
  const int N = static_cast<int>(vertex_indices.size());
  geom.positions.reserve(N);
  geom.generators.reserve(N);
  for (int i = 0; i < N; ++i) {
    geometry::MeshVertex::ConstPtr vertex = mesh.get_vertex(vertex_indices[i]);
    CROBOT_ASSERT(vertex != nullptr, "Invalid mesh vertex index.");
    math::Vector3d n = vertex->get_normal_inward().normalized();
    geom.positions.push_back(vertex->get_pos());
    auto [t1, t2] = compute_tangent_frame(n);
    geom.generators.push_back(compute_generators(n, t1, t2, mu, K));
  }
  return geom;
}

// Build the 6 x (N*K) wrench matrix W.
// Column [i*K + k] = [g_{ik}; r_i x g_{ik}].
math::MatrixXd build_wrench_matrix(const ContactGeometry& geom) {
  const int N      = static_cast<int>(geom.positions.size());
  const int K      = static_cast<int>(geom.generators[0].size());
  math::MatrixXd W = math::MatrixXd::Zero(6, N * K);
  for (int i = 0; i < N; ++i) {
    const math::Vector3d& r = geom.positions[i];
    for (int k = 0; k < K; ++k) {
      const math::Vector3d& g     = geom.generators[i][k];
      W.block<3, 1>(0, i * K + k) = g;
      W.block<3, 1>(3, i * K + k) = r.cross(g);
    }
  }
  return W;
}

// ============================================================================
// NLP (34) helpers: build the constant dual matrices A and b for LP duality.
//
// Variables in the primal (19): f_i (3D contact forces), split as f_i = f_i+
// - f_i-.  Primal variable vector f has dimension 6k.
// b (6k) and A (6k x n_dual) with n_dual = 12 + (1+l)*k, l = #cone edges.
//
// The dual problem is:
//   max_{u,g}  c(u)^T g
//   s.t.       A g <= b             (n_dual constraints)
//              g >= 0
//              u . u = 1
//              u . a_hat >= cos(alpha)
// ============================================================================

// Build b: [n1, -n1, ..., nk, -nk],  size 6k.
math::VectorXd build_dual_b(const ContactGeometry& geom) {
  const int k = static_cast<int>(geom.positions.size());
  math::VectorXd b(6 * k);
  for (int i = 0; i < k; ++i) {
    // normal n_i is the generator common direction:
    // each generator is n + mu*(cos*t1 + sin*t2) but n = average direction.
    // We recover n from the cross product structure: the first generator
    // gives g0 = n + mu*t1, so we need the stored generator mean.
    // However ContactGeometry only stores generators, not normals explicitly.
    // We recompute: n = sum(g) / K normalized to unit — but that only works
    // if mu > 0. Instead, build n from n = (1/(1+mu^2)) * sum(g·n)*n
    // Actually n is not stored. To avoid re-passing it we note:
    // g_{i,k} = n_i + mu*(cos*t1+sin*t2), so sum_k g_{i,k} / K = n_i
    // because cos/sin sum to zero over a regular polygon. So:
    const int l      = static_cast<int>(geom.generators[i].size());
    math::Vector3d n = math::Vector3d::Zero();
    for (int j = 0; j < l; ++j)
      n += geom.generators[i][j];
    n /= static_cast<double>(l);
    // n is now the inward unit normal (mu-scaled tangential parts cancel).
    b.segment<3>(6 * i)     = n;
    b.segment<3>(6 * i + 3) = -n;
  }
  return b;
}

// Build A: 6k x n_dual matrix A = [A1, -A1, A2, -A2, A3, A4].
// n_dual = 12 + (1+l)*k.
math::MatrixXd build_dual_A(const ContactGeometry& geom) {
  const int k      = static_cast<int>(geom.positions.size());
  const int l      = static_cast<int>(geom.generators[0].size());
  const int n_dual = 12 + (1 + l) * k;

  math::MatrixXd A = math::MatrixXd::Zero(6 * k, n_dual);

  // --- A1 (6k x 3): force-balance columns for f+ direction ---
  // A1 = block_diag of [I3; -I3] repeated k times (eq 25-26 combined)
  // A1^T f >= u_hat  =>  column of A corresponding to A1 is A1 itself.
  math::MatrixXd A1 = math::MatrixXd::Zero(6 * k, 3);
  for (int i = 0; i < k; ++i) {
    A1.block<3, 3>(6 * i, 0)     = math::Matrix3d::Identity();
    A1.block<3, 3>(6 * i + 3, 0) = -math::Matrix3d::Identity();
  }
  A.block(0, 0, 6 * k, 3) = A1;  // A1
  A.block(0, 3, 6 * k, 3) = -A1; // -A1

  // --- A2 (6k x 3): torque-balance columns (eq 27-28) ---
  // row block i: [-[ri]x; [ri]x] (3x3 each), stacked vertically.
  math::MatrixXd A2 = math::MatrixXd::Zero(6 * k, 3);
  for (int i = 0; i < k; ++i) {
    const math::Vector3d& r = geom.positions[i];
    // skew-symmetric [r]x such that [r]x * v = r x v
    math::Matrix3d rx;
    rx << 0, -r(2), r(1), r(2), 0, -r(0), -r(1), r(0), 0;
    A2.block<3, 3>(6 * i, 0)     = -rx;
    A2.block<3, 3>(6 * i + 3, 0) = rx;
  }
  A.block(0, 6, 6 * k, 3) = A2;
  A.block(0, 9, 6 * k, 3) = -A2;

  // --- A3 (6k x k): compression constraints (eq 29) ---
  // Column j: [n_j; -n_j; 0; ...; 0]
  for (int i = 0; i < k; ++i) {
    const int l_i    = static_cast<int>(geom.generators[i].size());
    math::Vector3d n = math::Vector3d::Zero();
    for (int j = 0; j < l_i; ++j)
      n += geom.generators[i][j];
    n /= static_cast<double>(l_i);
    A.block<3, 1>(6 * i, 12 + i)     = n;
    A.block<3, 1>(6 * i + 3, 12 + i) = -n;
  }

  // --- A4 (6k x k*l): friction cone generators (eq 30) ---
  // B_i columns are [r_{i,j} x r_{i,j+1}; -r_{i,j} x r_{i,j+1}] pairs.
  // Constraint: f_i . (r_{i,j} x r_{i,j+1}) >= 0
  // In the dual A4 block: column [i*l + j] gets the generator cross product
  // placed in rows 6i to 6i+5.
  int col_base = 12 + k;
  for (int i = 0; i < k; ++i) {
    const auto& gens = geom.generators[i];
    for (int j = 0; j < l; ++j) {
      math::Vector3d cross = gens[j].cross(gens[(j + 1) % l]);
      A.block<3, 1>(6 * i, col_base + i * l + j)     = cross;
      A.block<3, 1>(6 * i + 3, col_base + i * l + j) = -cross;
    }
  }

  return A;
}

// ============================================================================
// IPOPT NLP components for problem (34).
// Variables: x = [u (3), g (n_dual)]  with n_dual = 12 + (1+l)*k.
// ============================================================================

// Variable set: u (3) concatenated with g (n_dual).
class GraspQualityVars : public optimization::VariableSet {
public:
  GraspQualityVars(
      int n_dual,
      const math::Vector3d& u_init,
      const math::Vector3d& cone_axis,
      double cone_half_angle)
    : optimization::VariableSet(3 + n_dual, "grasp_vars"),
      m_n_dual(n_dual),
      m_cone_axis(cone_axis.normalized()),
      m_cone_half_angle(cone_half_angle) {
    m_x           = math::VectorXd::Zero(3 + n_dual);
    m_x.head<3>() = u_init;
  }

  void set_variables(const math::VectorXd& x) override {
    m_x = x;
  }
  math::VectorXd get_values() const override {
    return m_x;
  }

  optimization::VecBound get_bounds() const override {
    using namespace optimization;
    VecBound b(3 + m_n_dual, NoBound);
    // u unconstrained (sphere constraint is added separately)
    // g >= 0
    for (int i = 3; i < 3 + m_n_dual; ++i)
      b[i] = BoundGreaterZero;
    return b;
  }

private:
  int m_n_dual;
  math::Vector3d m_cone_axis;
  double m_cone_half_angle;
  math::VectorXd m_x;
};

// Cost term: minimize -(c(u)^T g) = -(u^T (g[0:3] - g[3:6]))
class GraspQualityCost : public optimization::CostTerm {
public:
  GraspQualityCost() : optimization::CostTerm("grasp_cost", {"grasp_vars"}) {
  }

  double get_cost(const math::VectorXd& x) const override {
    math::Vector3d u  = x.head<3>();
    math::Vector3d gp = x.segment<3>(3); // g[0:3]
    math::Vector3d gm = x.segment<3>(6); // g[3:6]
    return -u.dot(gp - gm);
  }

  // Analytical gradient:
  // d/du  = -(gp - gm)
  // d/dgp = -u
  // d/dgm = +u
  // d/dg_rest = 0
  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() == "grasp_vars") {
      math::VectorXd x    = varset->get_values();
      math::Vector3d u    = x.head<3>();
      math::Vector3d gp   = x.segment<3>(3);
      math::Vector3d gm   = x.segment<3>(6);
      math::Vector3d diff = gp - gm;
      for (int j = 0; j < 3; ++j)
        jac.coeffRef(0, j) = -diff(j);
      for (int j = 0; j < 3; ++j)
        jac.coeffRef(0, 3 + j) = -u(j);
      for (int j = 0; j < 3; ++j)
        jac.coeffRef(0, 6 + j) = +u(j);
    }
  }
};

// Constraint: u·u - 1 = 0  (unit-sphere equality)
class UnitSphereConstraint : public optimization::ConstraintSet {
public:
  UnitSphereConstraint()
    : optimization::ConstraintSet(1, "unit_sphere", {"grasp_vars"}) {
  }

  math::VectorXd get_values(const math::VectorXd& x) const override {
    math::Vector3d u = x.head<3>();
    math::VectorXd v(1);
    v(0) = u.dot(u) - 1.0;
    return v;
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(1, optimization::BoundZero);
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() == "grasp_vars") {
      math::Vector3d u = varset->get_values().head<3>();
      for (int j = 0; j < 3; ++j)
        jac.coeffRef(0, j) = 2.0 * u(j);
    }
  }
};

// Constraint: u·a_hat - cos(alpha/2) >= 0  (cone direction constraint)
class ConeDirectionConstraint : public optimization::ConstraintSet {
public:
  ConeDirectionConstraint(const math::Vector3d& axis, double half_angle)
    : optimization::ConstraintSet(1, "cone_dir", {"grasp_vars"}),
      m_axis(axis.normalized()),
      m_cos_alpha(std::cos(half_angle)) {
  }

  math::VectorXd get_values(const math::VectorXd& x) const override {
    math::Vector3d u = x.head<3>();
    math::VectorXd v(1);
    v(0) = u.dot(m_axis) - m_cos_alpha;
    return v;
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(1, optimization::BoundGreaterZero);
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() == "grasp_vars") {
      for (int j = 0; j < 3; ++j)
        jac.coeffRef(0, j) = m_axis(j);
    }
  }

private:
  math::Vector3d m_axis;
  double m_cos_alpha;
};

// Constraint: A*g <= b  i.e.  A*g - b <= 0.
// This is linear in g (and constant in u).  Dimension: 6k rows.
class DualFeasConstraint : public optimization::ConstraintSet {
public:
  DualFeasConstraint(const math::MatrixXd& A_mat, const math::VectorXd& b_vec)
    : optimization::ConstraintSet(
        static_cast<int>(b_vec.size()), "dual_feas", {"grasp_vars"}),
      m_A(A_mat),
      m_b(b_vec) {
  }

  math::VectorXd get_values(const math::VectorXd& x) const override {
    // x = [u(3), g(n_dual)]
    math::VectorXd g = x.tail(m_A.cols());
    return m_A * g - m_b; // should be <= 0
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(m_b.size(), optimization::BoundSmallerZero);
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() == "grasp_vars") {
      // jac has shape (6k) x (3 + n_dual)
      // d(Ag - b)/d[u,g] = [0 | A]
      for (int row = 0; row < m_A.rows(); ++row) {
        for (int col = 0; col < m_A.cols(); ++col) {
          double val = m_A(row, col);
          if (val != 0.0)
            jac.coeffRef(row, 3 + col) = val;
        }
      }
    }
  }

private:
  math::MatrixXd m_A;
  math::VectorXd m_b;
};

} // namespace

//==============================================================================
EquilibriumGrasp::EquilibriumGrasp(
    geometry::MeshPatches::ConstPtr mesh, double mu, int num_generators)
  : m_mesh(std::move(mesh)), m_mu(mu), m_num_generators(num_generators) {
}

//==============================================================================
EquilibriumGrasp::Result EquilibriumGrasp::solve(
    const std::vector<int>& vertex_indices, bool verbose) const {
  return solve(vertex_indices, math::Vector3d::UnitZ(), verbose);
}

//==============================================================================
EquilibriumGrasp::Result EquilibriumGrasp::solve(
    const std::vector<int>& vertex_indices,
    const math::Vector3d& gravity_dir,
    bool verbose) const {
  CROBOT_ASSERT(m_mesh != nullptr, "Mesh patches pointer is null.");
  CROBOT_ASSERT(!vertex_indices.empty(), "At least one contact is required.");
  CROBOT_ASSERT(
      gravity_dir.norm() > 1e-9,
      "Gravity direction must be non-zero for equilibrium solve.");

  EquilibriumGrasp::Result result;

  const int N          = static_cast<int>(vertex_indices.size());
  const int K          = m_num_generators;
  const int n_vars     = N * K + 1;
  const int lambda_col = N * K;

  math::Vector3d u = gravity_dir.normalized();

  // Gather contact geometry and compute K cone generators per contact.
  ContactGeometry geom
      = build_contact_geometry(*m_mesh, vertex_indices, m_mu, K);
  math::MatrixXd W = build_wrench_matrix(geom);

  // Build 7-row constraint matrix (7 x n_vars):
  //   rows 0-5 : wrench columns from W (force + torque balance)
  //   row  6   : normalization  sum_ik alpha_{ik} = 1
  // The lambda column contributes -u to the force rows only.
  // Since g_{ik} dot n_i = 1 for all k, row 6 also fixes the total normal
  // force to 1, matching the normalization of the original formulation.
  math::MatrixXd A             = math::MatrixXd::Zero(7, n_vars);
  A.topLeftCorner(6, N * K)    = W;
  A.block<3, 1>(0, lambda_col) = -u;
  A.row(6).head(N * K).setOnes();

  // Equality constraints: rows 0-5 equal zero, row 6 equals one.
  optimization::VecBound con_bounds(7, optimization::BoundZero);
  con_bounds[6] = optimization::Bounds(1.0, 1.0);

  // All alpha_{ik} and lambda are non-negative.
  optimization::VecBound var_bounds(n_vars, optimization::BoundGreaterZero);

  // Objective: minimize -lambda  (i.e. maximize lambda).
  math::VectorXd c = math::VectorXd::Zero(n_vars);
  c(lambda_col)    = -1.0;

  // Initial guess: distribute weight uniformly across all generators.
  math::VectorXd x0 = math::VectorXd::Zero(n_vars);
  x0.head(N * K).setConstant(1.0 / static_cast<double>(N * K));

  optimization::LinearProgram lp(c, A, con_bounds, var_bounds, x0);

  optimization::ClpSolver clp;
  clp.set_log_level(verbose ? 2 : 0);
  clp.solve(lp);

  result.solver_status = clp.get_return_status();
  if (result.solver_status != optimization::ClpSolver::OPTIMAL) {
    return result;
  }

  math::VectorXd x = lp.get_variables()->get_values();
  result.lambda    = x(lambda_col);
  result.feasible  = result.lambda > 1e-6;

  // Reconstruct contact forces: f_i = sum_k alpha_{i,k} * g_{i,k}.
  result.forces.reserve(N);
  for (int i = 0; i < N; ++i) {
    math::Vector3d f = math::Vector3d::Zero();
    for (int k = 0; k < K; ++k) {
      f += x(i * K + k) * geom.generators[i][k];
    }
    result.forces.push_back(f);
  }

  return result;
}

//==============================================================================
bool EquilibriumGrasp::is_equilibrium_grasp(
    const std::vector<int>& vertex_indices,
    double lambda_tol,
    bool verbose) const {
  return is_equilibrium_grasp(
      vertex_indices, math::Vector3d::UnitZ(), lambda_tol, verbose);
}

//==============================================================================
bool EquilibriumGrasp::is_equilibrium_grasp(
    const std::vector<int>& vertex_indices,
    const math::Vector3d& gravity_dir,
    double lambda_tol,
    bool verbose) const {
  EquilibriumGrasp::Result result = solve(vertex_indices, gravity_dir, verbose);
  return result.feasible && result.lambda > lambda_tol;
}

//==============================================================================
double EquilibriumGrasp::get_quality(
    const std::vector<int>& vertex_indices, bool verbose) const {
  return get_quality(vertex_indices, math::Vector3d::UnitZ(), verbose);
}

//==============================================================================
double EquilibriumGrasp::get_quality(
    const std::vector<int>& vertex_indices,
    const math::Vector3d& gravity_dir,
    bool verbose) const {
  return solve(vertex_indices, gravity_dir, verbose).lambda;
}

//==============================================================================
bool EquilibriumGrasp::is_strong_equilibrium_grasp(
    const std::vector<int>& vertex_indices,
    const math::Vector3d& cone_axis,
    double cone_half_angle,
    int num_disk_samples,
    bool verbose) const {
  CROBOT_ASSERT(m_mesh != nullptr, "Mesh patches pointer is null.");
  CROBOT_ASSERT(!vertex_indices.empty(), "At least one contact is required.");
  CROBOT_ASSERT(cone_axis.norm() > 1e-9, "Cone axis must be non-zero.");
  CROBOT_ASSERT(
      cone_half_angle >= 0.0 && cone_half_angle < 1.5707963267948966,
      "Cone half-angle must be in [0, pi/2).");
  CROBOT_ASSERT(num_disk_samples >= 1, "At least one disk sample required.");

  // Build geometry once; W is reused across all LP solves.
  const int K = m_num_generators;
  ContactGeometry geom
      = build_contact_geometry(*m_mesh, vertex_indices, m_mu, K);
  math::MatrixXd W = build_wrench_matrix(geom);
  const int N_K    = static_cast<int>(vertex_indices.size()) * K;

  // Gravity uncertainty cone: axis a_hat, half-angle alpha.
  // The disk D is the cross-section of the cone at unit height:
  //   D = { a_hat + tan(alpha) * (cos(theta)*t1 + sin(theta)*t2) }
  // We approximate D's bounding circle with an s-gon whose vertices gamma_j
  // are tested individually per Eq. 14.
  math::Vector3d a               = cone_axis.normalized();
  auto [t1, t2]                  = compute_tangent_frame(a);
  double disk_r                  = std::tan(cone_half_angle);
  static constexpr double two_pi = 2.0 * 3.14159265358979323846;

  // Feasibility LP (Eq. 14): variables alpha_{ik} >= 0, no objective.
  // CLP reports OPTIMAL iff a feasible point exists.
  math::VectorXd c = math::VectorXd::Zero(N_K);
  optimization::VecBound var_bounds(N_K, optimization::BoundGreaterZero);

  for (int j = 0; j < num_disk_samples; ++j) {
    double theta = two_pi * j / num_disk_samples;
    math::Vector3d gamma
        = a + disk_r * (std::cos(theta) * t1 + std::sin(theta) * t2);

    // Constraints: W * alpha = [gamma; 0_3]
    //   rows 0-2: sum_ik alpha_{ik} * g_{ik}          = gamma  (force balance)
    //   rows 3-5: sum_ik alpha_{ik} * (r_i x g_{ik})  = 0      (torque balance)
    optimization::VecBound con_bounds(6, optimization::BoundZero);
    con_bounds[0] = optimization::Bounds(gamma(0), gamma(0));
    con_bounds[1] = optimization::Bounds(gamma(1), gamma(1));
    con_bounds[2] = optimization::Bounds(gamma(2), gamma(2));

    optimization::LinearProgram lp(c, W, con_bounds, var_bounds);
    optimization::ClpSolver clp;
    clp.set_log_level(verbose ? 2 : 0);
    clp.solve(lp);

    if (clp.get_return_status() != optimization::ClpSolver::OPTIMAL) {
      return false;
    }
  }
  return true;
}

//==============================================================================
EquilibriumGrasp::NlpResult EquilibriumGrasp::solve_quality_nlp(
    const std::vector<int>& vertex_indices,
    const math::Vector3d& cone_axis,
    double cone_half_angle,
    bool verbose) const {
  CROBOT_ASSERT(m_mesh != nullptr, "Mesh patches pointer is null.");
  CROBOT_ASSERT(!vertex_indices.empty(), "At least one contact is required.");
  CROBOT_ASSERT(cone_axis.norm() > 1e-9, "Cone axis must be non-zero.");
  CROBOT_ASSERT(
      cone_half_angle >= 0.0 && cone_half_angle <= 3.14159265358979323846,
      "Cone half-angle must be in [0, pi].");

  const int k = static_cast<int>(vertex_indices.size());
  const int l = m_num_generators;

  ContactGeometry geom
      = build_contact_geometry(*m_mesh, vertex_indices, m_mu, l);

  // Build the constant dual LP matrices.
  math::MatrixXd A_mat = build_dual_A(geom);
  math::VectorXd b_vec = build_dual_b(geom);

  const int n_dual = 12 + (1 + l) * k;

  // Initial u: the normalized cone axis.
  math::Vector3d u_init = cone_axis.normalized();

  // Build IPOPT problem.
  auto vars = std::make_shared<GraspQualityVars>(
      n_dual, u_init, u_init, cone_half_angle);
  auto cost     = std::make_shared<GraspQualityCost>();
  auto unit_sph = std::make_shared<UnitSphereConstraint>();
  auto cone_dir
      = std::make_shared<ConeDirectionConstraint>(u_init, cone_half_angle);
  auto dual_feas = std::make_shared<DualFeasConstraint>(A_mat, b_vec);

  optimization::Problem nlp;
  nlp.add_variable_set(vars);
  nlp.add_cost_set(cost);
  nlp.add_constraint_set(unit_sph);
  nlp.add_constraint_set(cone_dir);
  nlp.add_constraint_set(dual_feas);

  optimization::IpoptSolver solver;
  solver.set_option("jacobian_approximation", "exact");
  solver.set_option("linear_solver", "mumps");
  solver.set_option("print_level", verbose ? 5 : 0);
  solver.set_option("sb", "yes"); // suppress IPOPT banner
  solver.solve(nlp);

  NlpResult result;
  math::VectorXd x = nlp.get_variable_values();
  result.u         = x.head<3>().normalized();
  math::VectorXd g = x.tail(n_dual);
  // Quality = c(u)^T g = u^T(g[0:3] - g[3:6])
  result.quality  = result.u.dot(g.head<3>() - g.segment<3>(3));
  result.feasible = result.quality > 1e-6;
  return result;
}

//==============================================================================
EquilibriumGrasp::NlpResult EquilibriumGrasp::solve_quality_nlp(
    const std::vector<int>& vertex_indices, bool verbose) const {
  return solve_quality_nlp(
      vertex_indices, math::Vector3d::UnitZ(), 0.0, verbose);
}
