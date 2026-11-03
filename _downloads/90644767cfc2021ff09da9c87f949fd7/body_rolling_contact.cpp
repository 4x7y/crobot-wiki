#include "crobot/kdl/body_rolling_contact.hpp"

#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace kdl;
using namespace optimization;

//==============================================================================
class SurfParamVariable : public VariableSet {
public:
  SurfParamVariable(
      const math::Vector2d& bounds_u, const math::Vector2d& bounds_v)
    : VariableSet(2, "uv") {
    bounds.resize(2);
    bounds.at(0) = Bounds(bounds_u[0], bounds_u[1]);
    bounds.at(1) = Bounds(bounds_v[0], bounds_v[1]);
  }

  void set_variables(const math::VectorXd& value) override {
    m_value = value;
  }

  math::VectorXd get_values() const override {
    return m_value;
  }

  VecBound get_bounds() const override {
    return bounds;
  }

private:
  math::Vector2d m_value;
  VecBound bounds;
};
//==============================================================================
struct DistCost : public optimization::CostTerm
{
  DistCost(std::shared_ptr<const math::SurfacePatch> surface_patch)
    : CostTerm("dist_cost", {"uv"}) {
    point = math::Vector3d(0.0, 0.0, 0.0);
    surf  = surface_patch;
  }

  double get_cost(const math::VectorXd& x) const override {
    // returns the distance between the point and (u, v)
    math::Vector2d uv         = x;
    math::Vector3d surf_point = surf->evaluate(uv);
    math::Vector3d diff       = surf_point - point;
    return diff.dot(diff);
  }

  void fill_jacobian_block(
      Component::Ptr var_set, math::SparseJacobian& jac) const override {
    if (var_set->get_name() == "uv") {
      // TODO
      math::Vector2d uv         = var_set->get_values();
      math::Vector3d surf_point = surf->evaluate(uv);
      math::Vector3d diff       = surf_point - point;
      // get the derivatives of the surface at (u, v)
      math::Matrix32 derivs = surf->jacb(uv);
      math::Vector3d du     = derivs.col(0);
      math::Vector3d dv     = derivs.col(1);
      // fill the jacobian
      jac.coeffRef(0, 0) = 2 * diff.dot(du);
      jac.coeffRef(0, 1) = 2 * diff.dot(dv);
    }
  }

  math::Vector3d point;
  std::shared_ptr<const math::SurfacePatch> surf;
};

//==============================================================================
BodyRollingContact::BodyRollingContact(
    ContactModel contact_model,
    ConstRigidBodyPtr body1,
    math::SurfacePatchSwitch::Ptr surface1,
    math::Vector2d contact_point1,
    ConstRigidBodyPtr body2,
    math::SurfacePatchSwitch::Ptr surface2,
    math::Vector2d contact_point2,
    double mu)
  : BodyContact(
      contact_model,
      body1,
      std::make_shared<BodyFrame>(
          body1, surface1->get_frame(contact_point1).tform_flip_normal()),
      body2,
      std::make_shared<BodyFrame>(body2, surface2->get_frame(contact_point2)),
      mu),
    m_surface1(surface1->clone()),
    m_surface2(surface2->clone()),
    m_contact_point1(contact_point1),
    m_contact_point2(contact_point2) {

  m_xfrc = math::Vector3d::Zero();
  m_xtrq = math::Vector3d::Zero();

  m_uv_dot1 = math::Vector2d::Zero();
  m_uv_dot2 = math::Vector2d::Zero();

  m_qvel_addr1 = m_body1->get_support_qvel_addr();
  std::sort(m_qvel_addr1.begin(), m_qvel_addr1.end());
  m_qvel_addr2 = m_body2->get_support_qvel_addr();
  std::sort(m_qvel_addr2.begin(), m_qvel_addr2.end());

  update_status(true);

  math::Vector2d u_bounds1 = m_surface1->get_u_bounds();
  math::Vector2d v_bounds1 = m_surface1->get_v_bounds();
  math::Vector2d u_bounds2 = m_surface2->get_u_bounds();
  math::Vector2d v_bounds2 = m_surface2->get_v_bounds();

  m_problems.resize(2);

  // setup the optimization problem
  m_problems[0] = std::make_shared<optimization::Problem>();
  m_problems[0]->add_variable_set(
      std::make_shared<SurfParamVariable>(u_bounds1, v_bounds1));
  m_problems[0]->add_cost_set(std::make_shared<DistCost>(m_surface1));

  m_problems[1] = std::make_shared<optimization::Problem>();
  m_problems[1]->add_variable_set(
      std::make_shared<SurfParamVariable>(u_bounds2, v_bounds2));
  m_problems[1]->add_cost_set(std::make_shared<DistCost>(m_surface2));

  m_solver = std::make_shared<optimization::IpoptSolver>();
  m_solver->set_option("jacobian_approximation", "exact");
  m_solver->set_option("linear_solver", "mumps");
  m_solver->set_option("print_level", 0);
}

//==============================================================================
BodyRollingContact::Ptr BodyRollingContact::from_yaml_node(
    const YAML::Node& node, ConstRigidBodyTreePtr rbtree) {
  CROBOT_ASSERT_ERROR(node["type"].as<std::string>() == "BodyRollingContact");
  ConstRigidBodyPtr body1 = rbtree->get_body(node["body1"].as<std::string>());
  math::SurfacePatchSwitch::Ptr surface1
      = math::SurfacePatchSwitch::from_yaml_node(node["surface1"]);

  math::Vector2d uv_scale1 = surface1->get_uv_scale();
  math::Vector2d contact_point1(
      node["contact_point1"][0].as<double>() * uv_scale1[0],
      node["contact_point1"][1].as<double>() * uv_scale1[1]);

  ConstRigidBodyPtr body2 = rbtree->get_body(node["body2"].as<std::string>());
  math::SurfacePatchSwitch::Ptr surface2
      = math::SurfacePatchSwitch::from_yaml_node(node["surface2"]);
  math::Vector2d uv_scale2 = surface2->get_uv_scale();
  math::Vector2d contact_point2(
      node["contact_point2"][0].as<double>() * uv_scale2[0],
      node["contact_point2"][1].as<double>() * uv_scale2[1]);

  double mu = node["mu"].as<double>();

  return std::make_shared<BodyRollingContact>(
      ContactModel::POINT_CONTACT_WITH_FRICTION,
      body1,
      surface1,
      contact_point1,
      body2,
      surface2,
      contact_point2,
      mu);
}

//==============================================================================
math::Vector3d BodyRollingContact::get_xfrc() const {
  return m_xfrc;
}

//==============================================================================
math::Vector3d BodyRollingContact::get_xtrq() const {
  return m_xtrq;
}

//==============================================================================
void BodyRollingContact::update_xfrc(math::Vector3d xfrc) {
  m_xfrc = xfrc;
}

//==============================================================================
void BodyRollingContact::update_xtrq(math::Vector3d xtrq) {
  m_xtrq = xtrq;
}

//==============================================================================
bool BodyRollingContact::is_valid_rolling(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Vector3d xpos1 = get_xpos1(tforms);
  math::Vector3d xpos2 = get_xpos2(tforms);
  math::Matrix3d xmat1 = get_xmat1(tforms);
  math::Matrix3d xmat2 = get_xmat2(tforms);

  math::Vector3d xnorm1 = xmat1 * math::Vector3d::UnitZ();
  math::Vector3d xnorm2 = xmat2 * math::Vector3d::UnitZ();

  // check if xpos1 and xpos2 are on the same place
  if ((xpos1 - xpos2).norm() > 1e-10) {
    return false;
  }

  // check if xnorm1 and xnorm2 are parallel and pointing to the opposite
  // direction
  if (xnorm1.dot(xnorm2) > -1e-10) {
    return false;
  }

  return true;
}

//==============================================================================
void BodyRollingContact::set_contact_points(
    math::Vector2d& contact_point1, math::Vector2d& contact_point2) {
  m_contact_point1 = contact_point1;
  m_contact_point2 = contact_point2;
  update_frames();
}

//==============================================================================
void BodyRollingContact::move_contact_points(
    const math::Vector2d& vec1, const math::Vector2d& vec2) {
  m_contact_point1 += vec1;
  m_contact_point2 += vec2;
  update_frames();
}

//==============================================================================
void BodyRollingContact::update_contact_points_in_dir(
    const math::Vector2d& uv_dot1, const math::Vector2d& uv_dot2, double dt) {
  set_moving_dir(uv_dot1, uv_dot2);
  math::Vector2d uv1_new = uv_dot1 * dt + m_contact_point1;
  math::Vector2d uv2_new = uv_dot2 * dt + m_contact_point2;

  set_contact_points(uv1_new, uv2_new);

  // check for patch update
  bool updated1 = m_surface1->patch_update(uv1_new[0], uv1_new[1]);
  bool updated2 = m_surface2->patch_update(uv2_new[0], uv2_new[1]);

  if (updated1) {
    uv1_new
        = evaluate_uv(m_frame1->get_bpos(), 0, math::Vector2d(0.5, uv1_new[2]));
  }

  if (updated2) {
    uv2_new
        = evaluate_uv(m_frame2->get_bpos(), 1, math::Vector2d(0.5, uv2_new[2]));
  }

  if (updated1 || updated2) {
    // std::cout << "patch updated" << std::endl;
    set_contact_points(uv1_new, uv2_new);
  }
}

//==============================================================================
void BodyRollingContact::update_contact_points_by_xpos(
    const math::Vector3d& xpos, const std::vector<math::Matrix4d>& tforms) {
  math::Matrix3d xmat1 = m_body1->get_xmat(tforms);
  math::Matrix3d xmat2 = m_body2->get_xmat(tforms);
  math::Vector3d bpos1 = xmat1.transpose() * (xpos - m_body1->get_xpos(tforms));
  math::Vector3d bpos2 = xmat2.transpose() * (xpos - m_body2->get_xpos(tforms));

  math::Vector2d uv1 = evaluate_uv(bpos1, 0, m_contact_point1);
  math::Vector2d uv2 = evaluate_uv(bpos2, 1, m_contact_point2);

  set_contact_points(uv1, uv2);
}

//==============================================================================
math::Vector2d BodyRollingContact::evaluate_uv(
    const math::Vector3d& bpos,
    double surface_id,
    const math::Vector2d& uv_guess) const {
  // update cost term
  std::shared_ptr<DistCost> optcost_dist = std::dynamic_pointer_cast<DistCost>(
      m_problems[surface_id]->get_costs().get_component("dist_cost"));
  optcost_dist->point = bpos;

  // update variable initial guess
  std::shared_ptr<SurfParamVariable> optvars_surf_param
      = std::dynamic_pointer_cast<SurfParamVariable>(
          m_problems[surface_id]->get_variables()->get_component("uv"));
  optvars_surf_param->set_variables(uv_guess);

  // solve for optimization problem
  m_solver->solve(*m_problems[surface_id]);

  math::Vector2d uv_est;
  if (m_solver->get_return_status() == IpoptSolver::SUCCEEDED) {
    uv_est = m_problems[surface_id]->get_variable_values();
  }
  else {
    CROBOT_LOG_WARN(
        "closed point on surface cannot be solved for point detected = {}",
        bpos.transpose());
    uv_est = uv_guess;
  }

  return uv_est;
}

//==============================================================================
std::pair<math::Vector2d, math::Vector2d>
BodyRollingContact::get_contact_points() const {
  std::pair<math::Vector2d, math::Vector2d> result
      = std::make_pair(m_contact_point1, m_contact_point2);
  return result;
}

//==============================================================================
// math::MatrixX6 BodyRollingContact::get_contact_map_inverse() const {
//   // orientation of the contact frame in the object frame
//   math::Matrix3d rotm = m_frame1->get_tform().tform2rotm();
//   math::Vector3d bpos = m_frame1->get_tform().tform2trvec();

//   math::Matrix6d Ginv;
//   // transform into the contact frame
//   Ginv.block<3, 3>(0, 0) = rotm.transpose();
//   Ginv.block<3, 3>(0, 3) = -bpos.skew() * rotm.transpose();
//   Ginv.block<3, 3>(3, 0) = math::Matrix3d::Zero();
//   Ginv.block<3, 3>(3, 3) = rotm.transpose();

//   return get_selection_matrix() * Ginv;
// }

//==============================================================================
/*
 *  G_deriv = [ bvelr_c.cross() * rotm                                 0 ] * S.T
 *            [bvel.cross() * rotm + bpos.cross() * bvelr_c.cross() * rotm
 *                                                 bvelr_c.cross() * rotm]
 */

math::Matrix6X BodyRollingContact::get_contact_map_deriv() const {
  // orientation of the contact frame in the object frame
  math::Matrix3d rotm       = m_frame1->get_tform().tform2rotm();
  math::Vector3d bpos       = m_frame1->get_tform().tform2trvec();
  math::Matrix3d bpos_cross = bpos.skew();
  math::Matrix32 dcdx       = m_surface1->jacb(m_contact_point1);
  math::Vector3d bvel       = dcdx * m_uv_dot1;
  math::Matrix3d bvel_cross = bvel.skew();

  // compute bvelr_c numerically
  math::Vector3d bvelr_c       = get_bvelr_ctc1();
  math::Matrix3d bvelr_c_cross = bvelr_c.skew();

  math::Matrix6d G_dot;
  // transform into the body frame
  G_dot.block<3, 3>(0, 0) = bvelr_c_cross * rotm;
  G_dot.block<3, 3>(0, 3) = math::Matrix3d::Zero();
  G_dot.block<3, 3>(3, 0)
      = bvel_cross * rotm + bpos_cross * bvelr_c_cross * rotm;
  G_dot.block<3, 3>(3, 3) = bvelr_c_cross * rotm;

  math::Matrix6X ctc_map_deriv = G_dot * get_selection_matrix().transpose();
  return ctc_map_deriv;
}

//==============================================================================
// math::Vector6d BodyRollingContact::map_bwrench_to_cwrench(
//     const math::Vector6d& bwrench) const {
//   return get_selection_matrix().transpose() * get_contact_map_inverse()
//          * bwrench;
// }

//==============================================================================
std::pair<math::Vector2d, math::Vector2d> BodyRollingContact::get_moving_dir(
    const std::vector<math::Matrix4d>& tforms,
    const math::VectorXd& qvel) const {
  math::Jacobian jacb1 = m_body1->get_jacb(tforms);
  math::Jacobian jacb2 = m_body2->get_jacb(tforms);

  math::Vector6d xvel_o = m_body1->get_xvel(jacb1, qvel);
  math::Vector6d xvel_m = m_body2->get_xvel(jacb2, qvel);

  // math::Matrix3d xmat_o = m_body1->get_xmat(tforms);
  // math::Matrix3d xmat_m = m_body2->get_xmat(tforms);

  math::Vector3d xvelr_o = xvel_o.head<3>();
  math::Vector3d xvelr_m = xvel_m.head<3>();

  // math::Vector3d xvelp_o = xvel_o.tail<3>();
  // math::Vector3d xvelp_m = xvel_m.tail<3>();

  // math::Vector3d ctc_xvel_o
  //     = xvelp_o
  //       + xvelr_o.skew() * xmat_o
  //             * m_surface1->get_frame(m_contact_point1).tform2trvec();
  // math::Vector3d ctc_xvel_m
  //     = xvelp_m
  //       + xvelr_m.skew() * xmat_m
  //             * m_surface2->get_frame(m_contact_point2).tform2trvec();

  // return get_moving_dir(
  //     tforms, xvel_o.head<3>(), xvel_m.head<3>(), ctc_xvel_m - ctc_xvel_o);

  // math::Vector3d xvelp_diff
  //     = ctc_xvel_m - ctc_xvel_o
  //       + (xmat_m * m_surface2->get_frame(m_contact_point2).tform2rotm()
  //              * m_surface2->jacb(m_contact_point2) * m_uv_dot2
  //          - xmat_o * m_surface1->get_frame(m_contact_point1).tform2rotm()
  //                * m_surface1->jacb(m_contact_point1) * m_uv_dot1);

  return get_moving_dir(tforms, xvelr_o, xvelr_m, math::Vector3d::Zero());
}

//==============================================================================
std::pair<math::Vector2d, math::Vector2d>
BodyRollingContact::get_rolling_moving_dir(
    const std::vector<math::Matrix4d>& tforms,
    const math::VectorXd& qvel) const {
  math::Jacobian jacb1 = m_body1->get_jacb(tforms);
  math::Jacobian jacb2 = m_body2->get_jacb(tforms);

  math::Vector6d xvel_o = m_body1->get_xvel(jacb1, qvel);
  math::Vector6d xvel_m = m_body2->get_xvel(jacb2, qvel);

  math::Vector3d xvelr_o = xvel_o.head<3>();
  math::Vector3d xvelr_m = xvel_m.head<3>();

  return get_moving_dir(tforms, xvelr_o, xvelr_m);
}

//==============================================================================
std::pair<math::Vector2d, math::Vector2d> BodyRollingContact::get_moving_dir(
    const std::vector<math::Matrix4d>& tforms,
    const math::Vector3d& xvelr_o,
    const math::Vector3d& xvelr_m,
    const math::Vector3d& xvelp_diff) const {

  (void)xvelp_diff;

  // stack the angular velocities
  math::Vector6d xvelr;
  xvelr << xvelr_o, xvelr_m;
  // orientation of the body frame
  math::Matrix3d xmat_o = m_body1->get_xmat(tforms);
  math::Matrix3d xmat_m = m_body2->get_xmat(tforms);

  // contact normals in the corresponding body frame
  math::Vector3d norm_o = m_surface1->normal(m_contact_point1);
  math::Vector3d norm_m = m_surface2->normal(m_contact_point2);

  // partial derivatives of the contact point in the corresponding body frame
  math::Matrix32 dcdx_o = m_surface1->jacb(m_contact_point1);
  math::Matrix32 dcdx_m = m_surface2->jacb(m_contact_point2);

  // partial derivatives of the contact normal in the corresponding body frame
  math::Matrix32 dndx_o = m_surface1->normal_jacb(m_contact_point1);
  math::Matrix32 dndx_m = m_surface2->normal_jacb(m_contact_point2);

  // (2.12) in Cole's paper of rolling contact
  math::Matrix6X A    = math::Matrix6X::Zero(6, 4);
  A.block<3, 2>(0, 0) = xmat_o * dcdx_o;

  A.block<3, 2>(0, 2) = -xmat_m * dcdx_m;
  A.block<3, 2>(3, 0) = xmat_o * dndx_o;
  A.block<3, 2>(3, 2) = xmat_m * dndx_m;

  // std::cout << "A in rolling contact: " << std::endl << A << std::endl;
  // std::cout << "xmat_o = " << std::endl << xmat_o << std::endl;
  // std::cout << "dcdx_o = " << std::endl << dcdx_o << std::endl;

  math::Matrix6d Bmat    = math::Matrix6d::Zero();
  Bmat.block<3, 3>(3, 0) = (xmat_o * norm_o).skew();
  Bmat.block<3, 3>(3, 3) = (xmat_m * norm_m).skew();

  // // if sliding happens
  // math::Vector6d vel_diff = math::Vector6d::Zero();
  // vel_diff.segment<3>(0)  = xvelp_diff;

  // if (m_body2->get_name() == "rh_ffdistal")
  //   std::cout << "ff norm: " << (xvelp_diff).norm() << std::endl;

  // if (m_body2->get_name() == "rh_thdistal")
  //   std::cout << "th norm: " << (xvelp_diff).norm() << std::endl;

  // if ((xvelp_diff).norm() < 1e-3 * 0.5) {
  //   vel_diff = math::Vector6d::Zero();
  // }

  math::Vector6d B = Bmat * xvelr; // + vel_diff;

  // solve Ax = B
  // math::Vector4d x = A.colPivHouseholderQr().solve(B);

  math::MatrixXd A_pinv = math::calculate_jacb_inv(A, 0.0001);
  math::Vector4d x      = A_pinv * B;

  // std::cout << "A_pinv in rolling contact: " << std::endl
  //           << A_pinv << std::endl;

  // std::cout << "B in rolling contact: " << std::endl
  //           << (xmat_o * norm_o).skew() << std::endl;

  // check if the solution is valid
  // std::cout << "A*x - B=" << A * x - B << std::endl;

  return std::make_pair(x.segment<2>(0), x.segment<2>(2));
}

//==============================================================================
void BodyRollingContact::set_moving_dir(
    const math::Vector2d& uv_dot1, const math::Vector2d& uv_dot2) {
  m_uv_dot1 = uv_dot1;
  m_uv_dot2 = uv_dot2;
}

//==============================================================================
// math::Vector3d BodyRollingContact::get_moving_xdir(
//     const std::vector<math::Matrix4d>& tforms,
//     const math::Vector3d& xvelr_o,
//     const math::Vector3d& xvelr_m) const {

//   CROBOT_LOG_ERROR("not implemented yet");
//   // math::Vector2d dir_o, dir_m;
//   // std::tie(dir_o, dir_m) = get_moving_dir(tforms, xvelr_o, xvelr_m);

//   // // partial derivatives of the contact point in the corresponding body
//   frame
//   // math::Matrix32 dcdx_o = m_surface1->jacb(m_contact_point1);
//   // math::Matrix32 dcdx_m = m_surface2->jacb(m_contact_point2);

//   // // orientation of the body frames
//   // math::Matrix3d xmat_o = m_body1->get_xmat(tforms);
//   // math::Matrix3d xmat_m = m_body2->get_xmat(tforms);

//   // math::Vector3d xvel_o = xmat_o * dcdx_o * dir_o;
//   // math::Vector3d xvel_m = xmat_m * dcdx_m * dir_m;

//   // // std::cout << "xvel_o: " << xvel_o.transpose() << std::endl;
//   // // std::cout << "xvel_m: " << xvel_m.transpose() << std::endl;

//   // CROBOT_ASSERT(xvel_o.isApprox(xvel_m, 1e-5), "xvel_o is not equal to
//   // xvel_m");

//   // math::Vector3d xvel_o_norm = xvel_o.normalized();

//   return math::Vector3d::Zero();
// }

//==============================================================================
math::Vector3d BodyRollingContact::get_bvelr_ctc1() const {
  math::Matrix32 drdx       = m_surface1->jacr(m_contact_point1);
  math::Vector3d bvelr_ctc1 = drdx * m_uv_dot1;
  return bvelr_ctc1;
}

//==============================================================================
math::Vector3d BodyRollingContact::get_xvelr_ctc1(
    const std::vector<math::Matrix4d>& tforms) const {
  math::Matrix3d xmat       = m_body1->get_xmat(tforms);
  math::Vector3d bvel       = get_bvelr_ctc1();
  math::Vector3d xvelr_ctc1 = xmat * bvel;
  return xvelr_ctc1;
}

//==============================================================================
void BodyRollingContact::update_frames() {
  math::Matrix4d tform1
      = m_surface1->get_frame(m_contact_point1).tform_flip_normal();
  m_frame1->set_tform(tform1);

  math::Matrix4d tform2 = m_surface2->get_frame(m_contact_point2);
  m_frame2->set_tform(tform2);
}
//==============================================================================
math::Matrix32 BodyRollingContact::get_jacb1() const {
  return m_surface1->jacb(m_contact_point1);
}

//==============================================================================
math::Matrix32 BodyRollingContact::get_jacb2() const {
  return m_surface2->jacb(m_contact_point2);
}

//==============================================================================
BodyRollingContact::Ptr BodyRollingContact::clone() const {
  return std::make_shared<BodyRollingContact>(
      get_contact_model(),
      m_body1,
      m_surface1,
      math::Vector2d(m_contact_point1[0], m_contact_point1[1]),
      m_body2,
      m_surface2,
      math::Vector2d(m_contact_point2[0], m_contact_point2[1]),
      get_mu());
}

//==============================================================================
math::SurfacePatchSwitch::Ptr BodyRollingContact::get_surface1() const {
  return m_surface1;
}

//==============================================================================
math::SurfacePatchSwitch::Ptr BodyRollingContact::get_surface2() const {
  return m_surface2;
}

//==============================================================================
bool BodyRollingContact::is_boundary_contact() const {

  bool is_boundary1 = m_surface1->is_boundary(m_contact_point1);
  bool is_boundary2 = m_surface2->is_boundary(m_contact_point2);

  CROBOT_ASSERT_WARNING(
      is_boundary1 == false, "contact point on obj is on the boundary");

  CROBOT_ASSERT_WARNING(
      is_boundary2 == false, "contact point on finger is on the boundary");

  return is_boundary1 || is_boundary2;
}

//==============================================================================
YAML::Node BodyRollingContact::to_yaml_node() const {
  YAML::Node node;
  node["type"]     = "BodyRollingContact";
  node["body1"]    = m_body1->get_name();
  node["surface1"] = m_surface1->to_yaml_node();

  math::Vector2d uv_scale1 = m_surface1->get_uv_scale();
  node["contact_point1"]   = std::vector<double>{
      m_contact_point1[0] / uv_scale1(0), m_contact_point1[1] / uv_scale1(1)};

  node["body2"]    = m_body2->get_name();
  node["surface2"] = m_surface2->to_yaml_node();

  math::Vector2d uv_scale2 = m_surface2->get_uv_scale();
  node["contact_point2"]   = std::vector<double>{
      m_contact_point2[0] / uv_scale2(0), m_contact_point2[1] / uv_scale2(1)};
  node["mu"] = get_mu();
  return node;
}
