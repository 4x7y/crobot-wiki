#include <crobot/ik/pose_refiner.hpp>

#include "crobot/common/common.hpp"
#include "crobot/ik/gik.hpp"
#include "crobot/ik/variable.hpp"
#include "crobot/ik/vars_surface_patch.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
PoseRefiner::PoseRefiner(const kdl::ConstRigidBodyTreePtr rbtree)
  : m_rbtree(rbtree),
    m_pose_varset(std::make_shared<ObjectPoseVariables>("pose")) {
  m_nlp.add_variable_set(m_pose_varset);
}

//==============================================================================
bool PoseRefiner::solve(
    const math::VectorXd& initial_pose,
    int max_iter,
    double max_time,
    bool verbose) {
  optimization::IpoptSolver ipopt;
  ipopt.set_option("linear_solver", "mumps");
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

    if (i > 0) {

      set_init_pose(
          initial_pose + math::VectorXd::Random(initial_pose.size()) * 0.0001);
    }

    ipopt.solve(m_nlp);
    if (ipopt.get_return_status() == optimization::IpoptSolver::SUCCEEDED) {
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

  return ipopt.get_return_status() == optimization::IpoptSolver::SUCCEEDED;
}

//==============================================================================
void PoseRefiner::show_details() const {
  m_nlp.print_current();
}

//==============================================================================
void PoseRefiner::validate_all() const {
  m_nlp.get_constraints().print_all();

  bool success = true;

  auto components = m_nlp.get_constraints().get_components();
  auto costs      = m_nlp.get_costs().get_components();

  components.insert(components.end(), costs.begin(), costs.end());

  for (auto component : components) {
    auto con
        = std::dynamic_pointer_cast<optimization::ConstraintSet>(component);
    math::MatrixXd jac_exact   = con->get_jacobian().toDense();
    math::MatrixXd jac_numdiff = con->get_jacobian_finite_diff().toDense();

    if ((jac_exact - jac_numdiff).norm() > 1e-6) {
      CROBOT_LOG_WARN(
          "ERROR: Jacobian is not correct for \"{}\"", con->get_name());
      math::MatrixXd diff = jac_exact - jac_numdiff;
      for (int i = 0; i < diff.rows(); i++) {
        for (int j = 0; j < diff.cols(); j++) {
          if (std::abs(diff(i, j)) > 1e-6) {
            CROBOT_LOG_WARN("diff({}, {}) = {}", i, j, diff(i, j));
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
void PoseRefiner::set_qpos(const math::VectorXd& qpos) {
  for (std::size_t i = 0; i < m_cons_body_ctc.size(); i++) {
    m_cons_body_ctc[i]->set_qpos(qpos);
  }

  if (m_cost_ctc.size() > 0) {
    for (std::size_t i = 0; i < m_cost_ctc.size(); i++) {
      m_cost_ctc[i]->set_qpos(qpos);
    }
  }
}

//==============================================================================
void PoseRefiner::set_init_pose(const math::VectorXd& pose) {
  m_pose_varset->set_values_all(pose);
}

//==============================================================================
void PoseRefiner::set_init_uv(
    const std::string& var_name, const math::Vector2d& uv) {
  for (auto varset : m_patch1_varsets) {
    if (varset->get_name() == var_name) {
      varset->set_values_all(uv);
      return;
    }
  }

  for (auto varset : m_patch2_varsets) {
    if (varset->get_name() == var_name) {
      varset->set_values_all(uv);
      return;
    }
  }
}

//==============================================================================
math::VectorXd PoseRefiner::get_pose() {
  return m_pose_varset->get_values_all();
}

//==============================================================================
std::pair<optimization::VariableSet::Ptr, optimization::VariableSet::Ptr>
PoseRefiner::add_body_contact_constraint(
    const std::string& name,
    const std::string& body_obj,
    const std::shared_ptr<const math::SurfacePatch>& patch_obj,
    const std::string& body_rob,
    const std::shared_ptr<const math::SurfacePatch>& patch_rob,
    double tolerance_p,
    double tolerance_n) {

  std::string patch_obj_varname = name + "_obj";
  std::string patch_rob_varname = name + "_rob";

  SurfacePatchVariables::Ptr varset_obj
      = std::make_shared<SurfacePatchVariables>(patch_obj_varname, patch_obj);
  SurfacePatchVariables::Ptr varset_rob
      = std::make_shared<SurfacePatchVariables>(patch_rob_varname, patch_rob);

  m_nlp.add_variable_set(varset_obj);
  m_nlp.add_variable_set(varset_rob);

  BodyContactConstraint::Ptr cons = std::make_shared<BodyContactConstraint>(
      name,
      m_rbtree,
      body_obj,
      patch_obj,
      patch_obj_varname,
      body_rob,
      patch_rob,
      patch_rob_varname,
      tolerance_p,
      tolerance_n);

  m_nlp.add_constraint_set(cons);

  m_cons_body_ctc.push_back(cons);
  m_patch1_varsets.push_back(varset_obj);
  m_patch2_varsets.push_back(varset_rob);

  return std::make_pair(varset_obj, varset_rob);
}

//==============================================================================
void PoseRefiner::add_plane_contact_constraint(
    const std::string& name,
    const math::Vector3d& bpos,
    const math::Vector3d& point,
    const math::Vector3d& normal,
    double tolerance) {
  m_cons_plane_ctc = std::make_shared<PlaneContactConstraint>(
      name, bpos, point, normal, tolerance);
  m_nlp.add_constraint_set(m_cons_plane_ctc);
}

//==============================================================================
void PoseRefiner::add_point_contact_constraint(
    const std::string& name,
    const math::Vector3d& bpos,
    const math::Vector3d& xpos,
    double tolerance) {
  m_cons_point_ctc
      = std::make_shared<PointContactConstraint>(name, bpos, xpos, tolerance);
  m_nlp.add_constraint_set(m_cons_point_ctc);
}

//==============================================================================
void PoseRefiner::add_surface_params_constraint(
    const std::string& name,
    const std::string& varname,
    const math::Vector2d& uv_est,
    double tolerance) {
  SurfaceParamsConstraint::Ptr surf_params_cons
      = std::make_shared<SurfaceParamsConstraint>(
          name, varname, uv_est, tolerance);
  m_surf_params_cons.push_back(surf_params_cons);
  m_nlp.add_constraint_set(surf_params_cons);
}

//==============================================================================
void PoseRefiner::set_plane_point(const math::Vector3d& point) {
  if (m_cons_plane_ctc)
    m_cons_plane_ctc->set_point(point);
  else
    CROBOT_LOG_ERROR("Plane contact constraint is not added yet.");
}

//==============================================================================

void PoseRefiner::set_plane_normal(const math::Vector3d& normal) {
  if (m_cons_plane_ctc)
    m_cons_plane_ctc->set_normal(normal);
  else
    CROBOT_LOG_ERROR("Plane contact constraint is not added yet.");
}

//==============================================================================
void PoseRefiner::add_potential_body_pose_cost(const math::VectorXd& pose_est) {
  m_cost_body_pose_potential = std::make_shared<BodyPosePotentialCost>(
      "body_pose_potential_cost", pose_est);
  m_nlp.add_cost_set(m_cost_body_pose_potential);
}

//==============================================================================
void PoseRefiner::add_potential_body_contact_cost(
    const std::string& name,
    const std::string& body_obj,
    const std::shared_ptr<const math::SurfacePatch>& patch_obj,
    const std::string& body_rob,
    const std::shared_ptr<const math::SurfacePatch>& patch_rob,
    const math::Vector3d& xpos_screw) {
  std::string patch_obj_varname = name + "_obj";
  std::string patch_rob_varname = name + "_rob";

  SurfacePatchVariables::Ptr varset_obj
      = std::make_shared<SurfacePatchVariables>(patch_obj_varname, patch_obj);
  SurfacePatchVariables::Ptr varset_rob
      = std::make_shared<SurfacePatchVariables>(patch_rob_varname, patch_rob);

  m_nlp.add_variable_set(varset_obj);
  m_nlp.add_variable_set(varset_rob);

  BodyContactPotentialCost::Ptr cost
      = std::make_shared<BodyContactPotentialCost>(
          name,
          m_rbtree,
          body_obj,
          patch_obj,
          patch_obj_varname,
          body_rob,
          patch_rob,
          patch_rob_varname,
          xpos_screw);

  m_nlp.add_cost_set(cost);

  m_cost_ctc.push_back(cost);
  m_patch1_varsets.push_back(varset_obj);
  m_patch2_varsets.push_back(varset_rob);
}
//==============================================================================

//==============================================================================
void PoseRefiner::set_cost_pose_est(const math::VectorXd& xpose_est) {
  if (m_cost_body_pose_potential)
    m_cost_body_pose_potential->set_body_xpos_est(xpose_est);
  else
    CROBOT_LOG_ERROR("Potential body pose cost is not added yet.");
}

//==============================================================================
void PoseRefiner::set_point_ctc(const math::Vector3d& xpos) {
  if (m_cons_point_ctc)
    m_cons_point_ctc->set_xpos(xpos);

  if (m_cost_ctc.size() > 0) {
    for (auto cost : m_cost_ctc) {
      cost->set_screw_xpos(xpos);
    }
  }
}

//==============================================================================
void PoseRefiner::set_surf_params(const std::vector<math::Vector2d>& uv) {
  if (m_surf_params_cons.size() == uv.size()) {
    for (size_t i = 0; i < uv.size(); i++) {
      m_surf_params_cons[i]->set_uv_est(uv[i]);
    }
  }
  else
    CROBOT_LOG_ERROR("Surface parameters wrong dimension.");
}
