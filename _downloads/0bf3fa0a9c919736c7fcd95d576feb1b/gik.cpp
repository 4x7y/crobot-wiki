#include "crobot/ik/gik.hpp"

#include <experimental/filesystem>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/cons_aiming.hpp"
#include "crobot/ik/cons_ball_joint.hpp"
#include "crobot/ik/cons_bounding_box.hpp"
#include "crobot/ik/cons_cartesian_target.hpp"
#include "crobot/ik/cons_distance_target.hpp"
#include "crobot/ik/cons_joint_angle.hpp"
#include "crobot/ik/cons_joint_coupling.hpp"
#include "crobot/ik/cons_obj_contact.hpp"
#include "crobot/ik/cons_opposite_target.hpp"
#include "crobot/ik/cons_orientation_target.hpp"
#include "crobot/ik/cons_orientation_vector.hpp"
#include "crobot/ik/cons_surface_contact.hpp"
#include "crobot/ik/cons_surface_curve_contact.hpp"
#include "crobot/ik/cons_surface_point_contact.hpp"
#include "crobot/ik/cost_qpos_quadratic_potential.hpp"
#include "crobot/ik/cost_qpos_ushape_potential.hpp"
#include "crobot/ik/variable.hpp"
#include "crobot/ik/vars_curve_segment.hpp"
#include "crobot/ik/vars_surface_patch.hpp"
#include "crobot/optimization/constraint_set.hpp"
#include "crobot/optimization/cost_term.hpp"
#include "crobot/optimization/optimization.hpp"
#include "crobot/optimization/variable_set.hpp"

using namespace crobot;
using namespace ik;
using namespace optimization;

//==============================================================================
bool LoadBoolean(
    const YAML::Node& node, const std::string& field_name, bool default_value) {
  if (node[field_name].IsDefined()) {
    return node[field_name].as<bool>();
  }
  else {
    return default_value;
  }
}

//==============================================================================
double LoadDouble(const YAML::Node& constraint, const std::string& field_name) {
  CROBOT_ASSERT_ERROR(
      constraint[field_name].IsDefined(),
      "Failed to load field '%s' for constraint '%s'",
      field_name.c_str(),
      constraint["name"].as<std::string>().c_str());
  const YAML::Node& field = constraint[field_name];
  return field.as<double>();
}

//==============================================================================
double LoadDouble(
    const YAML::Node& constraint,
    const std::string& field_name,
    double default_value) {
  if (constraint[field_name].IsDefined()) {
    const YAML::Node& field = constraint[field_name];
    return field.as<double>();
  }
  else {
    return default_value;
  }
}

//==============================================================================
math::Vector3d LoadVector3d(
    const YAML::Node& constraint, const std::string& field_name) {
  CROBOT_ASSERT_ERROR(
      constraint[field_name].IsDefined(),
      "Failed to load field '%s' for constraint '%s'",
      field_name.c_str(),
      constraint["name"].as<std::string>().c_str());
  const YAML::Node& field = constraint[field_name];
  if (field.IsSequence()) {
    CROBOT_ASSERT_ERROR(field.size() == 3, "Wrong dimension for Vector3d.");
    return math::Vector3d(
        field[0].as<double>(), field[1].as<double>(), field[2].as<double>());
  }
  else {
    return math::Vector3d(
        field["x"].as<double>(),
        field["y"].as<double>(),
        field["z"].as<double>());
  }
}

//==============================================================================
math::Vector3d LoadVector3d(
    const YAML::Node& constraint,
    const std::string& field_name,
    const math::Vector3d& default_value) {
  if (constraint[field_name].IsDefined()) {
    const YAML::Node& field = constraint[field_name];
    return math::Vector3d(
        field["x"].as<double>(),
        field["y"].as<double>(),
        field["z"].as<double>());
  }
  else {
    return default_value;
  }
}

//==============================================================================
math::VectorXd LoadVectorXd(
    const YAML::Node& constraint, const std::string& field_name) {
  CROBOT_ASSERT_ERROR(
      constraint[field_name].IsDefined(),
      "Failed to load field '%s' for constraint '%s'",
      field_name.c_str(),
      constraint["name"].as<std::string>().c_str());
  const YAML::Node& field = constraint[field_name];
  CROBOT_ASSERT_ERROR(
      field.IsScalar() || field.IsSequence(),
      "Failed to load field '%s' for constraint '%s'",
      field_name.c_str(),
      constraint["name"].as<std::string>().c_str());

  if (field.IsScalar()) {
    return math::Vector1d(field.as<double>());
  }
  else if (field.IsSequence()) {
    math::VectorXd v = math::VectorXd(field.size());
    for (size_t i = 0; i < field.size(); ++i) {
      v(i) = field[i].as<double>();
    }
  }

  return math::VectorXd();
}

//==============================================================================
std::string LoadString(
    const YAML::Node& constraint, const std::string& field_name) {
  CROBOT_ASSERT_ERROR(
      constraint[field_name].IsDefined(),
      "Failed to load field '%s' for constraint '%s'",
      field_name.c_str(),
      constraint["name"].as<std::string>().c_str());
  return constraint[field_name].as<std::string>();
}

//==============================================================================
std::shared_ptr<GeneralizedIK> GeneralizedIK::LoadFromFile(
    kdl::ConstRigidBodyTreePtr rbtree, const std::string& filename) {
  YAML::Node n = YAML::LoadFile(filename);
  CROBOT_ASSERT_ERROR(
      n.IsDefined(), "Failed to load file: %s", filename.c_str());
  return LoadFromYamlNode(rbtree, n);
}

//==============================================================================
std::shared_ptr<GeneralizedIK> GeneralizedIK::LoadFromYamlNode(
    kdl::ConstRigidBodyTreePtr rbtree, const YAML::Node& node) {
  return LoadFromYamlNode(
      rbtree,
      node,
      std::unordered_map<std::string, math::Geometry::ConstPtr>());
}

//==============================================================================
std::shared_ptr<GeneralizedIK> GeneralizedIK::LoadFromYamlNode(
    kdl::ConstRigidBodyTreePtr rbtree,
    const YAML::Node& n,
    const std::unordered_map<std::string, math::Geometry::ConstPtr>& geoms) {
  YAML::Node n_problem = n["problem"];
  CROBOT_ASSERT_ERROR(
      n_problem.IsDefined(), "Failed to load problem from file");
  CROBOT_ASSERT_ERROR(
      n_problem["name"].IsDefined(), "Failed to load problem name");
  std::string problem_name = n_problem["name"].as<std::string>();
  bool is_qpos_limited     = LoadBoolean(n_problem, "qpos_limited", true);

  auto load_patch = [&](const YAML::Node& node, const std::string patch_name) {
    std::string surface_name = LoadString(node[patch_name], "surface_name");
    auto geom                = geoms.find(surface_name);
    CROBOT_ASSERT_ERROR(
        geom != geoms.end(),
        "Geom \"%s\" was not found in the given geoms set.",
        surface_name.c_str());
    auto surface = std::dynamic_pointer_cast<const math::Surface>(geom->second);
    CROBOT_ASSERT_ERROR(
        surface != nullptr,
        "Geom \"%s\" is not a Surface geometry.",
        surface_name.c_str());
    return std::make_shared<math::SurfacePatch>(surface, node[patch_name]);
  };

  auto load_curve = [&](const YAML::Node& node,
                        const std::string segment_name) {
    std::string curve_name = LoadString(node[segment_name], "curve_name");
    auto geom              = geoms.find(curve_name);
    CROBOT_ASSERT_ERROR(
        geom != geoms.end(),
        "Geom \"%s\" was not found in the given geoms set.",
        curve_name.c_str());
    auto curve = std::dynamic_pointer_cast<const math::Curve>(geom->second);
    CROBOT_ASSERT_ERROR(
        curve != nullptr,
        "Geom \"%s\" is not a Curve geometry.",
        curve_name.c_str());
    return std::make_shared<math::CurveSegment>(curve, node[segment_name]);
  };

  // Construct an empty IK problem
  std::shared_ptr<GeneralizedIK> problem
      = std::make_shared<GeneralizedIK>(rbtree, is_qpos_limited);

  // Load constraints
  CROBOT_ASSERT_ERROR(
      n_problem["constraint"].IsDefined(),
      "Constraints not defined in the problem: %s",
      problem_name.c_str());
  YAML::Node constraints = n_problem["constraint"];
  CROBOT_ASSERT_ERROR(
      constraints.IsSequence(),
      "Constraints are not a sequence in the problem: %s",
      problem_name.c_str());

  for (YAML::const_iterator it = constraints.begin(); it != constraints.end();
       ++it) {
    const YAML::Node& constraint = *it;
    CROBOT_ASSERT_ERROR(
        constraint["name"].IsDefined(),
        "Failed to load constraint name in the problem: %s",
        problem_name.c_str());
    std::string name = constraint["name"].as<std::string>();
    CROBOT_ASSERT_ERROR(
        constraint["type"].IsDefined(),
        "Failed to load type for constraint '%s' in the problem: %s",
        name.c_str(),
        problem_name.c_str());
    std::string type = constraint["type"].as<std::string>();

    // BODY_POINT_POSITION
    if (type == "BODY_POINT_POSITION") {
      std::string body = LoadString(constraint, "body");
      math::Vector3d bpos
          = LoadVector3d(constraint, "bpos", math::Vector3d::Zero());
      math::Vector3d xpos = LoadVector3d(constraint, "xpos");
      problem->add_position_target(name, body, bpos, xpos);
    }
    // BODY_VECTOR_DIRECTION
    else if (type == "BODY_VECTOR_DIRECTION") {
      std::string body = LoadString(constraint, "body");
      math::Vector3d bvec
          = LoadVector3d(constraint, "bvec", math::Vector3d::UnitZ());
      math::Vector3d xvec = LoadVector3d(constraint, "xvec");
      double tol          = LoadDouble(constraint, "tolerance", 0.0);
      problem->add_orientation_vector_target(name, body, bvec, xvec, tol);
    }
    // BODY_POINT_BOUNDING_BOX
    else if (type == "BODY_POINT_BOUNDING_BOX") {
      std::string body       = LoadString(constraint, "body");
      math::Vector3d bpos    = LoadVector3d(constraint, "bpos");
      math::Vector3d xpos_lb = LoadVector3d(constraint, "xpos_lb");
      math::Vector3d xpos_ub = LoadVector3d(constraint, "xpos_ub");
      problem->add_cartesian_target(name, body, bpos, xpos_lb, xpos_ub);
    }
    // JOINT_POSITION
    else if (type == "JOINT_POSITION") {
      std::string joint   = LoadString(constraint, "joint");
      math::VectorXd qpos = LoadVectorXd(constraint, "qpos");
      double tol          = LoadDouble(constraint, "tolerance", 0.0);
      problem->add_joint_position_constraint(name, joint, qpos(0), tol);
    }
    // FIX_JOINT_POSITION
    else if (type == "FIX_JOINT_POSITION") {
      std::string joint   = LoadString(constraint, "joint");
      math::VectorXd qpos = LoadVectorXd(constraint, "qpos");
      problem->fix_joint(joint, qpos(0));
    }
    // BODY_POINT_DISTANCE
    else if (type == "BODY_POINT_DISTANCE") {
      std::string body = LoadString(constraint, "body");
      math::Vector3d bpos
          = LoadVector3d(constraint, "bpos", math::Vector3d::Zero());
      math::Vector3d xpos = LoadVector3d(constraint, "xpos");
      double distance     = LoadDouble(constraint, "distance");
      double tol          = LoadDouble(constraint, "tolerance", 0.0);
      problem->add_distance_target(name, body, bpos, xpos, distance, tol);
    }
    else if (type == "BODY_VECTOR_AIMING") {
      std::string body = LoadString(constraint, "body");
      math::Vector3d bpos
          = LoadVector3d(constraint, "bpos", math::Vector3d::Zero());
      math::Vector3d bvec
          = LoadVector3d(constraint, "bvec", math::Vector3d::UnitZ());
      math::Vector3d xpos = LoadVector3d(constraint, "xpos");
      double tol          = LoadDouble(constraint, "tolerance", 0.0);
      problem->add_aiming_target(name, body, bpos, bvec, xpos, tol);
    }
    else if (type == "JOINT_COUPLING") {
      std::string joint1 = LoadString(constraint, "joint1");
      std::string joint2 = LoadString(constraint, "joint2");
      double coeff1      = LoadDouble(constraint, "coeff1");
      double coeff2      = LoadDouble(constraint, "coeff2");
      double tol         = LoadDouble(constraint, "tolerance", 0.0);
      problem->add_joint_coupling_constraint(
          name, joint1, coeff1, joint2, coeff2, tol);
    }
    else if (type == "SURFACE_SURFACE_CONTACT") {
      std::string body1 = LoadString(constraint, "body1");
      std::string body2 = LoadString(constraint, "body2");
      double tol        = LoadDouble(constraint, "tolerance", 0.0);
      std::shared_ptr<const math::SurfacePatch> patch1
          = load_patch(constraint, "patch1");
      std::shared_ptr<const math::SurfacePatch> patch2
          = load_patch(constraint, "patch2");
      problem->add_surface_contact_constraint(
          name, body1, patch1, body2, patch2, tol);
    }
    else if (type == "SURFACE_CURVE_CONTACT") {
      std::string body1 = LoadString(constraint, "body1");
      std::string body2 = LoadString(constraint, "body2");
      double tol        = LoadDouble(constraint, "tolerance", 0.0);

      std::shared_ptr<const math::SurfacePatch> surface_patch
          = load_patch(constraint, "patch");
      std::shared_ptr<const math::CurveSegment> curve_segment
          = load_curve(constraint, "curve");
      problem->add_surface_curve_contact_constraint(
          name, body1, surface_patch, body2, curve_segment, tol);
    }
    else {
      CROBOT_ASSERT_ERROR(false, "Unknown constraint type: %s", type.c_str());
    }
    CROBOT_LOG_INFO("Added IK constraint {}: {}", type, name);
  }

  return problem;
}

//==============================================================================
GeneralizedIK::GeneralizedIK(
    const kdl::ConstRigidBodyTreePtr rbtree, bool limited)
  : m_rbtree(rbtree),
    m_qpos_varset(std::make_shared<QposVariables>(rbtree, limited, "qpos")) {
  m_nlp.add_variable_set(m_qpos_varset);
  m_solve_count = 0;
}

//==============================================================================
bool GeneralizedIK::solve(int max_iter, double max_time, bool verbose) {
  IpoptSolver ipopt;
  ipopt.set_option("linear_solver", "mumps"); // mumps, ma57
  // ipopt.set_option("jacobian_approximation", "finite-difference-values");
  ipopt.set_option("jacobian_approximation", "exact");
  // ipopt.set_option("derivative_test", "first-order");
  if (verbose) {
    ipopt.set_option("print_level", 4);
  }
  else {
    ipopt.set_option("print_level", 0);
  }
  ipopt.set_option("constr_viol_tol", 1e-6);
  ipopt.set_option("tol", 1e-6);
  ipopt.set_option("print_user_options", "no");
  ipopt.set_option("mu_strategy", "adaptive");
  ipopt.set_option("hessian_approximation", "limited-memory");
  ipopt.set_option("fixed_variable_treatment", "make_parameter");
  ipopt.set_option("warm_start_bound_frac", 1e-6);
  ipopt.set_option("warm_start_bound_push", 1e-6);
  ipopt.set_option("warm_start_mult_bound_push", 1e-6);
  ipopt.set_option("warm_start_slack_bound_frac", 1e-6);
  ipopt.set_option("warm_start_slack_bound_push", 1e-6);
  ipopt.set_option("max_cpu_time", 1.0);

  auto start = std::chrono::system_clock::now();
  for (int i = 0; i < max_iter; i++) {
    if (i > m_solve_count) {
      set_init_qpos(m_rbtree->make_random_qpos());
    }
    else if (i > 0) {
      set_init_qpos(
          m_rbtree->make_random_qpos_offset(m_qpos_ref, m_qpos_offset_limit));
    }

    ipopt.solve(m_nlp);
    if (ipopt.get_return_status() == IpoptSolver::SUCCEEDED) {
      break;
    }
    auto end = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
        > max_time) {
      break;
    }
  }

  if (verbose) {
    m_nlp.print_current();
  }

  return ipopt.get_return_status() == IpoptSolver::SUCCEEDED;
}

//==============================================================================
void GeneralizedIK::show_details() const {
  m_nlp.print_current();
}

//==============================================================================
void GeneralizedIK::validate_constraints() const {
  m_nlp.get_constraints().print_all();

  bool success    = true;
  auto components = m_nlp.get_constraints().get_components();
  for (auto component : components) {
    auto con
        = std::dynamic_pointer_cast<optimization::ConstraintSet>(component);
    math::MatrixXd jac_exact   = con->get_jacobian().toDense();
    math::MatrixXd jac_numdiff = con->get_jacobian_finite_diff().toDense();

    if ((jac_exact - jac_numdiff).norm() > 1e-4) {
      CROBOT_LOG_WARN(
          "ERROR: Jacobian is not correct for constraint \"{}\"",
          con->get_name());
      math::MatrixXd diff = jac_exact - jac_numdiff;
      for (int i = 0; i < diff.rows(); i++) {
        for (int j = 0; j < diff.cols(); j++) {
          if (std::abs(diff(i, j)) > 1e-4) {
            CROBOT_LOG_WARN(
                "diff({}, {}) = {} - {} = {}",
                i,
                j,
                jac_exact(i, j),
                jac_numdiff(i, j),
                diff(i, j));
            success = false;
          }
        }
      }
    }
  }

  if (success) {
    CROBOT_LOG_INFO("All Jacobians are correct.");
  }
}

//==============================================================================
void GeneralizedIK::set_init_qpos(const math::VectorXd& qpos) {
  m_qpos_varset->set_values_all(qpos);
}

//==============================================================================
void GeneralizedIK::set_init_guess_limit(
    const math::VectorXd& qpos_ref,
    const math::VectorXd& qpos_offset_limit,
    int solve_count) {
  m_qpos_ref          = qpos_ref;
  m_qpos_offset_limit = qpos_offset_limit;
  m_solve_count       = solve_count;
}

//==============================================================================
math::VectorXd GeneralizedIK::get_qpos() {
  return m_qpos_varset->get_values_all();
}

//==============================================================================
void GeneralizedIK::add_position_target(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& bpos,
    const math::Vector3d& xpos,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<CartesianConstraint>(
      name,
      m_rbtree,
      body,
      bpos,
      xpos - math::Vector3d::Ones() * tolerance,
      xpos + math::Vector3d::Ones() * tolerance));
}

//==============================================================================
void GeneralizedIK::add_position_target(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& xpos,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<CartesianConstraint>(
      name,
      m_rbtree,
      body,
      math::Vector3d::Zero(),
      xpos - math::Vector3d::Ones() * tolerance,
      xpos + math::Vector3d::Ones() * tolerance));
}

//==============================================================================
void GeneralizedIK::add_bounding_box_constraint(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& bpos,
    const std::string& body_bbox,
    const math::Vector3d& bpos_bbox,
    const math::Matrix3d& rotm_bbox,
    const math::Vector3d& size_bbox,
    double tolerance) {
  math::Matrix4d tform_bbox    = math::Matrix4d::Identity();
  tform_bbox.block<3, 1>(0, 3) = bpos_bbox;
  tform_bbox.block<3, 3>(0, 0) = rotm_bbox;

  m_nlp.add_constraint_set(std::make_shared<BoundingBoxConstraint>(
      name, m_rbtree, body, bpos, body_bbox, tform_bbox, size_bbox, tolerance));
}

//==============================================================================
void GeneralizedIK::add_distance_target(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& bpos,
    const math::Vector3d& xpos,
    double distance,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<DistanceConstraint>(
      name, m_rbtree, body, bpos, "world", xpos, distance, tolerance));
}

//==============================================================================
void GeneralizedIK::add_distance_target(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& xpos,
    double distance,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<DistanceConstraint>(
      name,
      m_rbtree,
      body,
      math::Vector3d::Zero(),
      "world",
      xpos,
      distance,
      tolerance));
}

//==============================================================================
void GeneralizedIK::add_distance_target(
    const std::string& name,
    const std::string& body1,
    const math::Vector3d& bpos1,
    const std::string& body2,
    const math::Vector3d& bpos2,
    double distance,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<DistanceConstraint>(
      name, m_rbtree, body1, bpos1, body2, bpos2, distance, tolerance));
}

//==============================================================================
void GeneralizedIK::add_opposite_orientation_target(
    const std::string& name,
    const std::string& body1,
    const math::Vector3d& bpos1,
    const math::Vector3d& bvec1,
    const std::string& body2,
    const math::Vector3d& bpos2,
    const math::Vector3d& bvec2,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<OppositeOrientationConstraint>(
      name, m_rbtree, body1, bpos1, bvec1, body2, bpos2, bvec2, tolerance));
}

//==============================================================================
void GeneralizedIK::add_cartesian_target(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& bpos,
    const math::Vector3d& xpos_lb,
    const math::Vector3d& xpos_ub) {
  m_nlp.add_constraint_set(std::make_shared<CartesianConstraint>(
      name, m_rbtree, body, bpos, xpos_lb, xpos_ub));
}

//==============================================================================
void GeneralizedIK::add_cartesian_target(
    const std::string& name,
    const std::string& body,
    const math::Vector3d& xpos_lb,
    const math::Vector3d& xpos_ub) {
  m_nlp.add_constraint_set(std::make_shared<CartesianConstraint>(
      name, m_rbtree, body, math::Vector3d::Zero(), xpos_lb, xpos_ub));
}

//==============================================================================
void GeneralizedIK::add_orientation_target(
    const std::string& con_name,
    const std::string& body_name,
    const math::Quaternion& xquat,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<OrientationTargetConstraint>(
      con_name, m_rbtree, body_name, xquat, tolerance));
}

//==============================================================================
void GeneralizedIK::add_orientation_vector_target(
    const std::string& con_name,
    const std::string& body_name,
    const math::Vector3d& bvec,
    const math::Vector3d& xvec,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<OrientationVectorConstraint>(
      con_name, m_rbtree, body_name, bvec, xvec, tolerance));
}

//==============================================================================
void GeneralizedIK::add_aiming_target(
    const std::string& con_name,
    const std::string& body_name,
    const math::Vector3d& xpos,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<AimingConstraint>(
      con_name,
      m_rbtree,
      body_name,
      math::Vector3d::Zero(),
      math::Vector3d::Zero(),
      xpos,
      tolerance));
}

//==============================================================================
void GeneralizedIK::add_aiming_target(
    const std::string& name,
    const std::string& body_name,
    const math::Vector3d& bvec,
    const math::Vector3d& xpos,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<AimingConstraint>(
      name,
      m_rbtree,
      body_name,
      math::Vector3d::Zero(),
      bvec,
      xpos,
      tolerance));
}

//==============================================================================
void GeneralizedIK::add_aiming_target(
    const std::string& name,
    const std::string& body_name,
    const math::Vector3d& bpos,
    const math::Vector3d& bvec,
    const math::Vector3d& xpos,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<AimingConstraint>(
      name, m_rbtree, body_name, bpos, bvec, xpos, tolerance));
}

//==============================================================================
void GeneralizedIK::add_joint_coupling_constraint(
    const std::string& name,
    const std::string& joint1_name,
    double coeff1,
    const std::string& joint2_name,
    double coeff2,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<JointCouplingConstraint>(
      name, m_rbtree, joint1_name, coeff1, joint2_name, coeff2, tolerance));
}

//==============================================================================
void GeneralizedIK::add_joint_position_constraint(
    const std::string& name,
    const std::string& joint_name,
    double qpos,
    double tolerance) {
  m_nlp.add_constraint_set(std::make_shared<JointAngleConstraint>(
      name, m_rbtree, joint_name, qpos, tolerance));
}

//==============================================================================
std::pair<optimization::VariableSet::Ptr, optimization::VariableSet::Ptr>
GeneralizedIK::add_surface_contact_constraint(
    const std::string& name,
    const std::string& body1,
    const std::shared_ptr<const math::SurfacePatch>& patch1,
    const std::string& body2,
    const std::shared_ptr<const math::SurfacePatch>& patch2,
    double tolerance) {

  std::string patch1_varname = name + "_patch1_params";
  std::string patch2_varname = name + "_patch2_params";

  optimization::VariableSet::Ptr varset1
      = std::make_shared<SurfacePatchVariables>(patch1_varname, patch1);
  optimization::VariableSet::Ptr varset2
      = std::make_shared<SurfacePatchVariables>(patch2_varname, patch2);

  m_nlp.add_variable_set(varset1);
  m_nlp.add_variable_set(varset2);
  m_nlp.add_constraint_set(std::make_shared<SurfaceContactConstraint>(
      name,
      m_rbtree,
      body1,
      patch1,
      patch1_varname,
      body2,
      patch2,
      patch2_varname,
      tolerance));

  return std::make_pair(varset1, varset2);
}

//==============================================================================
std::pair<optimization::VariableSet::Ptr, optimization::VariableSet::Ptr>
GeneralizedIK::add_surface_curve_contact_constraint(
    const std::string& name,
    const std::string& body1,
    const std::shared_ptr<const math::SurfacePatch>& patch,
    const std::string& body2,
    const std::shared_ptr<const math::CurveSegment>& segment,
    double tolerance) {

  std::string patch_varname   = name + "_surface_uv";
  std::string segment_varname = name + "_curve_u";

  optimization::VariableSet::Ptr varset1
      = std::make_shared<SurfacePatchVariables>(patch_varname, patch);
  optimization::VariableSet::Ptr varset2
      = std::make_shared<CurveSegmentVariable>(segment_varname, segment);

  m_nlp.add_variable_set(varset1);
  m_nlp.add_variable_set(varset2);
  m_nlp.add_constraint_set(std::make_shared<SurfaceCurveContactConstraint>(
      name,
      m_rbtree,
      body1,
      patch,
      patch_varname,
      body2,
      segment,
      segment_varname,
      tolerance));

  return std::make_pair(varset1, varset2);
}

//==============================================================================
void GeneralizedIK::add_unit_quaternion_constraint(
    const std::string& name, const std::string& joint_name, double tolerance) {
  CROBOT_ASSERT_ERROR(0.0 <= tolerance && tolerance <= 1.0);
  kdl::ConstRigidBodyPtr body = m_rbtree->get_body_by_joint(joint_name);
  CROBOT_ASSERT_ERROR(body != nullptr);
  CROBOT_ASSERT_ERROR(body->get_joint() != nullptr);
  CROBOT_ASSERT_ERROR(body->get_joint()->get_type() == kdl::JointType::BALL);
  m_nlp.add_constraint_set(
      std::make_shared<BallJointConstraint>(name, body, tolerance));
}

//==============================================================================
void GeneralizedIK::add_unit_quaternion_constraint(double tolerance) {
  CROBOT_ASSERT_ERROR(0.0 <= tolerance && tolerance <= 1.0);
  std::vector<kdl::ConstRigidBodyPtr> bodies = m_rbtree->get_bodies();
  for (kdl::ConstRigidBodyPtr body : bodies) {
    if (body->get_joint()
        && body->get_joint()->get_type() == kdl::JointType::BALL) {
      m_nlp.add_constraint_set(std::make_shared<BallJointConstraint>(
          body->get_joint()->get_name() + "_unit_quat", body, tolerance));
    }
  }
}

//==============================================================================
void GeneralizedIK::add_potential_cost(PotentialCost cost) {
  if (cost == QPOS_QUADRATIC_POTENTIAL) {
    m_nlp.add_cost_set(std::make_shared<QposQuadraticPotentialCost>(
        "qpos_quadratic_potential_cost", m_rbtree));
  }
  else if (cost == QPOS_USHAPE_POTENTIAL) {
    m_nlp.add_cost_set(std::make_shared<QposUshapePotentialCost>(
        "qpos_ushape_potential_cost", m_rbtree));
  }
  else {
    CROBOT_ASSERT_ERROR(false, "Unknown potential cost type");
  }
}

//==============================================================================
void GeneralizedIK::fix_joint_range(
    const std::pair<int, int>& range, const math::VectorXd& qpos) const {
  m_qpos_varset->fix_joint_range(range, qpos);
}

//==============================================================================
void GeneralizedIK::fix_joint(
    const std::string& joint_name, const math::VectorXd& qpos) const {
  m_qpos_varset->fix_joint(joint_name, qpos);
}

//==============================================================================
void GeneralizedIK::fix_joint(
    const std::string& joint_name, double qpos) const {
  CROBOT_ASSERT_ERROR(
      m_rbtree->get_body_by_joint(joint_name)->get_joint()->get_type()
              == kdl::JointType::PRISMATIC
          || m_rbtree->get_body_by_joint(joint_name)->get_joint()->get_type()
                 == kdl::JointType::REVOLUTE,
      "Only prismatic and revolute joints can be fixed with a scalar value.");
  fix_joint(joint_name, math::Vector1d(qpos));
}

//==============================================================================
void GeneralizedIK::fix_joint(
    const std::string& joint_name, const math::Quaternion& quat) const {
  CROBOT_ASSERT_ERROR(
      m_rbtree->get_body_by_joint(joint_name)->get_joint()->get_type()
          == kdl::JointType::BALL,
      "Only ball joints can be fixed with quaternion.");
  fix_joint(joint_name, math::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
}

//==============================================================================
ik::Constraint::Ptr GeneralizedIK::get_constriant(const std::string& name) {
  return m_nlp.get_constraints().get_component<ik::Constraint>(name);
}

//==============================================================================
void GeneralizedIK::add_contact_constraint(
    const std::string& name,
    const std::string& body1_name,
    const math::Matrix4d& body1_tform,
    const math::Matrix4d& body2_xform,
    double pos_tol,
    double orien_tol) {

  add_position_target(
      "contact_xpos_target_" + name,
      body1_name,
      body1_tform.tform2trvec(),
      body2_xform.tform2trvec(),
      pos_tol);
  add_orientation_vector_target(
      "contact_orin_vec_target_" + name,
      body1_name,
      body1_tform.block<3, 1>(0, 2),
      body2_xform.block<3, 1>(0, 2),
      orien_tol);
}

//==============================================================================
optimization::VariableSet::Ptr
GeneralizedIK::add_surface_point_contact_constraint(
    const std::string& name,
    const std::string& body1,
    const std::shared_ptr<const math::SurfacePatch>& patch1,
    const math::Vector3d& xpos_body2,
    const math::Vector3d& xvec_body2,
    double tolerance) {
  std::string patch1_varname = name + "_patch1_params";

  optimization::VariableSet::Ptr varset1
      = std::make_shared<SurfacePatchVariables>(patch1_varname, patch1);

  m_nlp.add_variable_set(varset1);

  m_nlp.add_constraint_set(std::make_shared<SurfacePointContactConstraint>(
      name,
      m_rbtree,
      body1,
      patch1,
      patch1_varname,
      xpos_body2,
      xvec_body2,
      tolerance));

  return varset1;
}

//==============================================================================
optimization::VariableSet::Ptr GeneralizedIK::add_object_contact_constraint(
    const std::string& name,
    const std::string& body_obj,
    const math::Vector3d& bpos_obj,
    const math::Vector3d& bvec_obj,
    const std::string& body_rob,
    std::shared_ptr<const math::SurfacePatch> patch_rob,
    double tolerance_p,
    double tolerance_n) {
  std::string patch_varname = name + "_patch_params";

  optimization::VariableSet::Ptr varset
      = std::make_shared<SurfacePatchVariables>(patch_varname, patch_rob);

  m_nlp.add_variable_set(varset);

  m_nlp.add_constraint_set(std::make_shared<ObjectContactConstraint>(
      name,
      m_rbtree,
      body_obj,
      bpos_obj,
      bvec_obj,
      body_rob,
      patch_rob,
      patch_varname,
      tolerance_p,
      tolerance_n));

  return varset;
}
