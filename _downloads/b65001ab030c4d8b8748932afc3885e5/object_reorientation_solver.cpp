#include <cmath>

#include "crobot/grasping/object_reorientation_solver.hpp"
#include "crobot/optimization/bounds.hpp"
#include "crobot/optimization/constraint_set.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/ipopt_solver.hpp"
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
std::vector<math::Vector3d> compute_generators(
    const math::Vector3d& n,
    const math::Vector3d& t1,
    const math::Vector3d& t2,
    double mu,
    int K) {
  std::vector<math::Vector3d> gens;
  gens.reserve(K);
  for (int k = 0; k < K; ++k) {
    double angle = math::constantsd::two_pi() * k / K;
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

// Rotate a direction vector with an axis-angle increment.
math::Vector3d rotate_by_axis_angle(
    const math::Vector3d& dir, const math::Vector3d& delta) {
  const double theta = delta.norm();
  if (theta < 1e-12) {
    return dir;
  }
  return math::AngleAxisd(theta, delta / theta) * dir;
}

class ReorientationVars : public optimization::VariableSet {
public:
  ReorientationVars(
      int num_contacts, const std::vector<math::Vector3d>& initial_forces)
    : optimization::VariableSet(3 + 3 * num_contacts, "reorient_vars"),
      m_num_contacts(num_contacts) {
    m_x = math::VectorXd::Zero(3 + 3 * m_num_contacts);
    for (int i = 0; i < m_num_contacts; ++i) {
      if (i < static_cast<int>(initial_forces.size())) {
        m_x.segment<3>(3 + 3 * i) = initial_forces[i];
      }
    }
  }

  void set_variables(const math::VectorXd& x) override {
    m_x = x;
  }

  math::VectorXd get_values() const override {
    return m_x;
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(
        3 + 3 * m_num_contacts, optimization::NoBound);
  }

private:
  int m_num_contacts;
  math::VectorXd m_x;
};

// Penalize orientation adjustment magnitude.
class ReorientationCost : public optimization::CostTerm {
public:
  ReorientationCost()
    : optimization::CostTerm("reorient_cost", {"reorient_vars"}) {
  }

  // objective: minimize ||Delta psi||^2.
  double get_cost(const math::VectorXd& x) const override {
    return x.head<3>().squaredNorm();
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() == "reorient_vars") {
      const math::Vector3d delta = varset->get_values().head<3>();
      for (int j = 0; j < 3; ++j)
        jac.coeffRef(0, j) = 2.0 * delta(j);
    }
  }
};

// Nonlinear force-equilibrium constraint.
class ReorientationForceBalanceConstraint : public optimization::ConstraintSet {
public:
  explicit ReorientationForceBalanceConstraint(int num_contacts)
    : optimization::ConstraintSet(
        3, "reorient_force_balance", {"reorient_vars"}),
      m_num_contacts(num_contacts),
      m_z0(math::Vector3d::UnitZ()),
      m_mg(1.0) {
  }

  void set_parameters(
      const math::Vector3d& z_axis_in_body, double gravity_force_magnitude) {
    m_z0 = z_axis_in_body.normalized();
    m_mg = gravity_force_magnitude;
  }

  // Force balance:
  // sum_i f_i - mg * z_hat(psi + Delta psi) = 0.
  math::VectorXd get_values(const math::VectorXd& x) const override {
    math::Vector3d sum_f = math::Vector3d::Zero();
    for (int i = 0; i < m_num_contacts; ++i) {
      sum_f += x.segment<3>(3 + 3 * i);
    }
    const math::Vector3d z_new = rotate_by_axis_angle(m_z0, x.head<3>());
    return sum_f - m_mg * z_new;
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(3, optimization::BoundZero);
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() != "reorient_vars") {
      return;
    }

    const math::VectorXd x     = varset->get_values();
    const math::Vector3d delta = x.head<3>();
    const double eps           = 1e-8;
    // Numerical derivative for rotated z direction w.r.t. delta_psi.
    for (int j = 0; j < 3; ++j) {
      math::Vector3d plus  = delta;
      math::Vector3d minus = delta;
      plus(j) += eps;
      minus(j) -= eps;
      const math::Vector3d z_plus  = rotate_by_axis_angle(m_z0, plus);
      const math::Vector3d z_minus = rotate_by_axis_angle(m_z0, minus);
      const math::Vector3d deriv   = (z_plus - z_minus) / (2.0 * eps);
      for (int r = 0; r < 3; ++r)
        jac.coeffRef(r, j) = -m_mg * deriv(r);
    }

    for (int i = 0; i < m_num_contacts; ++i) {
      for (int r = 0; r < 3; ++r) {
        jac.coeffRef(r, 3 + 3 * i + r) = 1.0;
      }
    }
  }

private:
  int m_num_contacts;
  math::Vector3d m_z0;
  double m_mg;
};

// Torque-equilibrium constraint.
class ReorientationTorqueConstraint : public optimization::ConstraintSet {
public:
  explicit ReorientationTorqueConstraint(
      const std::vector<math::Vector3d>& contacts)
    : optimization::ConstraintSet(
        3, "reorient_torque_balance", {"reorient_vars"}),
      m_contacts(contacts) {
  }

  // Torque balance:
  // sum_i c_i x f_i = 0.
  math::VectorXd get_values(const math::VectorXd& x) const override {
    math::Vector3d tau = math::Vector3d::Zero();
    for (int i = 0; i < static_cast<int>(m_contacts.size()); ++i) {
      const math::Vector3d fi = x.segment<3>(3 + 3 * i);
      tau += m_contacts[i].cross(fi);
    }
    return tau;
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(3, optimization::BoundZero);
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() != "reorient_vars") {
      return;
    }

    for (int i = 0; i < static_cast<int>(m_contacts.size()); ++i) {
      const math::Vector3d& c = m_contacts[i];
      math::Matrix3d cx;
      cx << 0, -c(2), c(1), c(2), 0, -c(0), -c(1), c(0), 0;
      for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
          jac.coeffRef(r, 3 + 3 * i + col) = cx(r, col);
        }
      }
    }
  }

private:
  std::vector<math::Vector3d> m_contacts;
};

// Polyhedral friction-cone facet constraints.
class ReorientationFrictionConstraint : public optimization::ConstraintSet {
public:
  explicit ReorientationFrictionConstraint(const ContactGeometry& geom)
    : optimization::ConstraintSet(
        static_cast<int>(geom.positions.size() * geom.generators[0].size()),
        "reorient_friction",
        {"reorient_vars"}) {
    const int N = static_cast<int>(geom.positions.size());
    const int L = static_cast<int>(geom.generators[0].size());
    m_edge_normals.resize(N);
    for (int i = 0; i < N; ++i) {
      m_edge_normals[i].reserve(L);
      for (int j = 0; j < L; ++j) {
        m_edge_normals[i].push_back(
            geom.generators[i][j].cross(geom.generators[i][(j + 1) % L]));
      }
    }
  }

  // Polyhedral friction-cone inequalities for each contact edge:
  // f_i · (xi_{i,j} x xi_{i,j+1}) >= 0.
  math::VectorXd get_values(const math::VectorXd& x) const override {
    const int N = static_cast<int>(m_edge_normals.size());
    const int L = static_cast<int>(m_edge_normals[0].size());
    math::VectorXd v(N * L);
    for (int i = 0; i < N; ++i) {
      const math::Vector3d fi = x.segment<3>(3 + 3 * i);
      for (int j = 0; j < L; ++j) {
        v(i * L + j) = fi.dot(m_edge_normals[i][j]);
      }
    }
    return v;
  }

  optimization::VecBound get_bounds() const override {
    return optimization::VecBound(get_rows(), optimization::BoundGreaterZero);
  }

  void fill_jacobian_block(
      optimization::Component::Ptr varset,
      math::SparseJacobian& jac) const override {
    if (varset->get_name() != "reorient_vars") {
      return;
    }

    const int N = static_cast<int>(m_edge_normals.size());
    const int L = static_cast<int>(m_edge_normals[0].size());
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < L; ++j) {
        const int row = i * L + j;
        for (int col = 0; col < 3; ++col) {
          jac.coeffRef(row, 3 + 3 * i + col) = m_edge_normals[i][j](col);
        }
      }
    }
  }

private:
  std::vector<std::vector<math::Vector3d>> m_edge_normals;
};

} // namespace

//==============================================================================
struct ObjectReorientationSolver::Impl
{
  std::vector<int> vertex_indices;
  ContactGeometry geom;
  int num_contacts = 0;
  bool prepared    = false;

  std::shared_ptr<ReorientationVars> vars;
  std::shared_ptr<ReorientationCost> cost;
  std::shared_ptr<ReorientationForceBalanceConstraint> force_balance;
  std::shared_ptr<ReorientationTorqueConstraint> torque_balance;
  std::shared_ptr<ReorientationFrictionConstraint> friction;

  optimization::Problem problem;
  optimization::IpoptSolver solver;
};

//==============================================================================
ObjectReorientationSolver::ObjectReorientationSolver(
    geometry::MeshPatches::ConstPtr mesh, double mu, int num_generators)
  : m_mesh(std::move(mesh)),
    m_mu(mu),
    m_num_generators(num_generators),
    m_impl(nullptr) {
}

ObjectReorientationSolver::~ObjectReorientationSolver() = default;

//==============================================================================
void ObjectReorientationSolver::ensure_impl() const {
  if (!m_impl) {
    m_impl = std::make_unique<Impl>();
    m_impl->solver.set_option("jacobian_approximation", "exact");
    m_impl->solver.set_option("linear_solver", "mumps");
    m_impl->solver.set_option("sb", "yes"); // suppress IPOPT banner
  }
}

//==============================================================================
void ObjectReorientationSolver::configure_contacts(
    const std::vector<int>& vertex_indices) const {
  CROBOT_ASSERT(m_mesh != nullptr, "Mesh patches pointer is null.");
  CROBOT_ASSERT(!vertex_indices.empty(), "At least one contact is required.");

  ensure_impl();
  if (m_impl->prepared && m_impl->vertex_indices == vertex_indices) {
    return;
  }

  m_impl->vertex_indices = vertex_indices;
  m_impl->geom
      = build_contact_geometry(*m_mesh, vertex_indices, m_mu, m_num_generators);
  m_impl->num_contacts = static_cast<int>(vertex_indices.size());

  std::vector<math::Vector3d> zero_forces(
      m_impl->num_contacts, math::Vector3d::Zero());
  m_impl->vars
      = std::make_shared<ReorientationVars>(m_impl->num_contacts, zero_forces);
  m_impl->cost          = std::make_shared<ReorientationCost>();
  m_impl->force_balance = std::make_shared<ReorientationForceBalanceConstraint>(
      m_impl->num_contacts);
  m_impl->torque_balance
      = std::make_shared<ReorientationTorqueConstraint>(m_impl->geom.positions);
  m_impl->friction
      = std::make_shared<ReorientationFrictionConstraint>(m_impl->geom);

  m_impl->problem = optimization::Problem();
  m_impl->problem.add_variable_set(m_impl->vars);
  m_impl->problem.add_cost_set(m_impl->cost);
  m_impl->problem.add_constraint_set(m_impl->force_balance);
  m_impl->problem.add_constraint_set(m_impl->torque_balance);
  m_impl->problem.add_constraint_set(m_impl->friction);
  m_impl->prepared = true;
}

//==============================================================================
ObjectReorientationSolver::Result ObjectReorientationSolver::solve(
    const std::vector<int>& vertex_indices,
    const math::Vector3d& z_axis_in_body,
    double gravity_force_magnitude,
    const std::vector<math::Vector3d>& initial_forces,
    bool verbose) const {
  configure_contacts(vertex_indices);
  return solve(
      z_axis_in_body, gravity_force_magnitude, initial_forces, verbose);
}

//==============================================================================
ObjectReorientationSolver::Result ObjectReorientationSolver::solve(
    const math::Vector3d& z_axis_in_body,
    double gravity_force_magnitude,
    const std::vector<math::Vector3d>& initial_forces,
    bool verbose) const {
  CROBOT_ASSERT(m_impl && m_impl->prepared, "Call configure_contacts() first.");
  CROBOT_ASSERT(
      z_axis_in_body.norm() > 1e-9, "z-axis direction must be non-zero.");
  CROBOT_ASSERT(
      gravity_force_magnitude >= 0.0,
      "Gravity force magnitude must be non-negative.");

  const int N       = m_impl->num_contacts;
  math::VectorXd x0 = m_impl->vars->get_values();
  if (x0.size() != 3 + 3 * N) {
    x0 = math::VectorXd::Zero(3 + 3 * N);
  }
  x0.head<3>().setZero();

  if (static_cast<int>(initial_forces.size()) == N) {
    for (int i = 0; i < N; ++i) {
      x0.segment<3>(3 + 3 * i) = initial_forces[i];
    }
  }
  else if (x0.tail(3 * N).norm() < 1e-12) {
    const math::Vector3d f_uniform
        = (gravity_force_magnitude / static_cast<double>(N))
          * z_axis_in_body.normalized();
    for (int i = 0; i < N; ++i) {
      x0.segment<3>(3 + 3 * i) = f_uniform;
    }
  }

  m_impl->vars->set_variables(x0);
  m_impl->force_balance->set_parameters(
      z_axis_in_body, gravity_force_magnitude);

  m_impl->solver.set_option("print_level", verbose ? 5 : 0);
  m_impl->solver.solve(m_impl->problem);

  Result result;
  result.solver_status = m_impl->solver.get_return_status();

  const math::VectorXd x = m_impl->problem.get_variable_values();
  result.delta_psi       = x.head<3>();
  result.objective       = result.delta_psi.squaredNorm();
  result.adjusted_z
      = rotate_by_axis_angle(z_axis_in_body.normalized(), result.delta_psi)
            .normalized();

  result.forces.reserve(N);
  for (int i = 0; i < N; ++i) {
    result.forces.push_back(x.segment<3>(3 + 3 * i));
  }

  const bool solved
      = result.solver_status == optimization::IpoptSolver::SUCCEEDED
        || result.solver_status
               == optimization::IpoptSolver::SOLVED_TO_ACCEPTABLE_LEVEL
        || result.solver_status
               == optimization::IpoptSolver::FEASIBLE_POINT_FOUND;
  result.feasible = solved;
  return result;
}
