#include "crobot/ik/jacobian_ik.hpp"

#include <cmath>
#include <iostream>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/ik/jacobian_ik_dof_target.hpp"
#include "crobot/ik/jacobian_ik_pose_target.hpp"

using namespace crobot;
using namespace ik;

//==============================================================================
JacobianIK::JacobianIK(
    kdl::ConstRigidBodyTreePtr rbtree,
    JacobianIKAlgorithm alg,
    int num_iters,
    double tol)
  : m_rbtree(rbtree),
    m_alg(alg),
    m_num_iters(num_iters),
    m_tol_squared(tol * tol),
    m_ncons(0),
    m_lm_bias_damping(0.01),
    m_lm_step_tol(tol) {
  m_qpos = m_rbtree->get_home_qpos();
}

//==============================================================================
void JacobianIK::fix_joint(
    const std::string& joint_name, const math::VectorXd& value) {
  kdl::ConstJointPtr joint      = m_rbtree->get_joint(joint_name);
  std::pair<int, int> qvel_addr = joint->get_qvel_addr()[0];
  for (int i = 0; i < qvel_addr.second; i++) {
    m_qvel_addr_fixed.insert({i + qvel_addr.first, value[i]});
    m_qvel_addr.erase(i + qvel_addr.first);
  }
}

//==============================================================================
void JacobianIK::fix_joint(const std::string& joint_name, double value) {
  kdl::ConstJointPtr joint      = m_rbtree->get_joint(joint_name);
  std::pair<int, int> qvel_addr = joint->get_qvel_addr()[0];
  CROBOT_ASSERT_ERROR(
      qvel_addr.second == 1,
      "Only single-DOF joints can be fixed in this function.");
  m_qvel_addr_fixed.insert({qvel_addr.first, value});
  m_qvel_addr.erase(qvel_addr.first);
}

//==============================================================================
void JacobianIK::fix_joint(const std::string& joint_name) {
  kdl::ConstJointPtr joint      = m_rbtree->get_joint(joint_name);
  std::pair<int, int> qvel_addr = joint->get_qvel_addr()[0];
  for (int i = 0; i < qvel_addr.second; i++) {
    m_qvel_addr_fixed.insert({i + qvel_addr.first, std::nullopt});
    m_qvel_addr.erase(i + qvel_addr.first);
  }
}

//==============================================================================
void JacobianIK::remove_fix_joint(const std::string& joint_name) {
  kdl::ConstJointPtr joint      = m_rbtree->get_joint(joint_name);
  std::pair<int, int> qvel_addr = joint->get_qvel_addr()[0];
  for (int i = 0; i < qvel_addr.second; i++) {
    m_qvel_addr.insert(i + qvel_addr.first);
    m_qvel_addr_fixed.erase(i + qvel_addr.first);
  }
}

//==============================================================================
void JacobianIK::fix_body(const std::string& body_name) {
  std::vector<int> qvel_addr
      = m_rbtree->get_body(body_name)->get_support_qvel_addr();
  for (std::size_t i = 0; i < qvel_addr.size(); i++) {
    m_qvel_addr_fixed.insert({qvel_addr[i], std::nullopt});
    m_qvel_addr.erase(qvel_addr[i]);
  }
}

//==============================================================================
void JacobianIK::add_target(
    kdl::ConstFramePtr desired_frame,
    const std::string& body_name,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm,
    double weight) {
  add_target(
      std::make_shared<JacobianIKPoseTarget>(
          desired_frame, m_rbtree->get_body(body_name), bpos, rotm),
      weight);
}

//==============================================================================
void JacobianIK::add_rolling_target_selected_dofs(
    kdl::ConstFramePtr desired_frame,
    const std::string& body_name,
    const std::vector<int>& dofs,
    kdl::BodyContact::ConstPtr contact,
    math::EulerAngleSeq euler_seq,
    double weight) {
  add_target(
      std::make_shared<JacobianIKDofTarget>(
          desired_frame,
          m_rbtree->get_body(body_name),
          dofs,
          contact,
          euler_seq),
      weight);
}

//==============================================================================
void JacobianIK::add_target_selected_dofs(
    kdl::ConstFramePtr desired_frame,
    const std::string& body_name,
    const std::vector<int>& dofs,
    math::EulerAngleSeq euler_seq,
    const math::Vector3d& bpos,
    const math::Matrix3d& rotm,
    double weight) {
  add_target(
      std::make_shared<JacobianIKDofTarget>(
          desired_frame,
          m_rbtree->get_body(body_name),
          dofs,
          euler_seq,
          bpos,
          rotm),
      weight);
}

//==============================================================================
void JacobianIK::add_target(
    std::shared_ptr<JacobianIKTarget> target, double weight) {
  CROBOT_ASSERT_ERROR(
      weight >= 0.0, "JacobianIK target weight must be non-negative.");

  m_targets.emplace_back(target, weight);
  m_ncons += target->get_ndofs();

  std::vector<int> qvel_addr = target->get_qvel_addrs();
  for (int addr : qvel_addr) {
    if (m_qvel_addr_fixed.find(addr) == m_qvel_addr_fixed.end()) {
      m_qvel_addr.insert(addr);
    }
  }
}

//==============================================================================
bool JacobianIK::solve(
    const math::VectorXd& qpos_guess, const math::VectorXd& qvel) {
  m_qpos = qpos_guess;
  for (const auto& qpos_fixed : m_qvel_addr_fixed) {
    if (qpos_fixed.second) {
      m_qpos[qpos_fixed.first] = qpos_fixed.second.value();
    }
  }

  std::vector<int> qvel_addr(m_qvel_addr.begin(), m_qvel_addr.end());
  math::VectorXd dx(m_ncons);
  math::MatrixXd jacb_full(m_ncons, m_rbtree->get_qvel_dim());
  math::MatrixXd eye
      = math::MatrixXd::Identity(qvel_addr.size(), qvel_addr.size());

  for (int i = 0; i < m_num_iters; i++) {
    math::VectorXd qs                  = m_qpos(qvel_addr);
    std::vector<math::Matrix4d> tforms = m_rbtree->forward_kinematics(m_qpos);

    int row = 0;
    for (const auto& weighted_target : m_targets) {
      const std::shared_ptr<JacobianIKTarget>& target
          = std::get<0>(weighted_target);
      double weight = std::get<1>(weighted_target);
      math::VectorXd dxi;
      math::MatrixXd jacbi;
      std::tie(dxi, jacbi) = target->compute_value_and_jacobian(tforms);
      int ndofs            = target->get_ndofs();
      double row_weight    = std::sqrt(weight);

      if (row_weight != 1.0) {
        dxi *= row_weight;
        jacbi *= row_weight;
      }

      dx.segment(row, ndofs)           = dxi;
      jacb_full.middleRows(row, ndofs) = jacbi;
      row += ndofs;
    }

    double error_squared = dx.squaredNorm();

    if (error_squared < m_tol_squared) {
      return true;
    }

    math::MatrixXd jacb = jacb_full(Eigen::all, qvel_addr);
    math::VectorXd dq;

    if (m_alg == JacobianIKAlgorithm::LEVENBERG_MARQUARDT) {
      double E_k = 0.5 * dx.squaredNorm();

      // g_k = J_k^T * e_k (gradient of objective)
      math::VectorXd g_k = jacb.transpose() * dx;

      // W_N = E_k * I_n + w_bar_N * I_n (error-adaptive damping)
      // where w_bar_N is the bias damping value
      int n_dofs         = jacb.cols();
      math::MatrixXd W_N = (E_k + m_lm_bias_damping)
                           * math::MatrixXd::Identity(n_dofs, n_dofs);

      // H_k = J_k^T * J_k + W_N (Gauss-Newton approximation with damping)
      math::MatrixXd H_k = jacb.transpose() * jacb + W_N;

      // Solve H_k * dq = g_k using LDLT with SVD fallback for robustness
      bool solve_success = false;

      // Try LDLT decomposition first
      Eigen::LDLT<math::MatrixXd> ldlt_solver(H_k);
      if (ldlt_solver.info() == Eigen::Success && ldlt_solver.isPositive()
          && H_k.determinant() != 0) {
        dq            = ldlt_solver.solve(g_k);
        solve_success = true;
      }

      // Fallback to SVD if LDLT fails or matrix is ill-conditioned
      if (!solve_success) {
        Eigen::JacobiSVD<math::MatrixXd> svd_solver(
            H_k, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double rcond_threshold = 1e-10;
        dq                     = svd_solver.solve(g_k);

        // Check condition number and warn if high
        double sigma_max = svd_solver.singularValues()(0);
        double sigma_min = svd_solver.singularValues()(
            svd_solver.singularValues().size() - 1);
        if (sigma_min > rcond_threshold * sigma_max) {
          solve_success = true;
        }
      }

      if (!solve_success) {
        CROBOT_LOG_WARN(
            "LM: Hessian solve may be ill-conditioned at iteration {}", i);
      }

      // A tiny LM step with a tiny weighted gradient indicates a stationary
      // point of the weighted least-squares objective, even if the residual is
      // non-zero because the targets are inconsistent.
      if (dq.norm() < m_lm_step_tol) {
        if (g_k.norm() < m_lm_step_tol) {
          return true;
        }

        if (error_squared > m_tol_squared) {
          CROBOT_LOG_WARN(
              "LM: Step stagnation detected at iteration {} with residual norm "
              "{}",
              i,
              std::sqrt(error_squared));
          return false;
        }

        return true;
      }
    }
    else if (m_alg == JacobianIKAlgorithm::SVD) {
      math::MatrixXd jacb_inv = math::calculate_jacb_inv(jacb, 0.001, true);

      dq = jacb_inv * dx;
    }
    else if (m_alg == JacobianIKAlgorithm::DAMPED_LEAST_SQUARE) {
      math::MatrixXd jacb_inv = math::calculate_jacb_inv(jacb);

      dq = [&]() -> math::VectorXd {
        if (m_null_space_obj != nullptr) {
          math::MatrixXd null = eye - jacb_inv * jacb;
          math::VectorXd dvdq = m_null_space_obj->get_weighted_partial(
              qpos_guess, qvel)(qvel_addr);

          math::VectorXd ret1 = jacb_inv * dx;
          math::VectorXd ret2 = null * dvdq;

          return ret1 + ret2;
          // return ret1;
        }
        // standard jacobian inverse IK
        else {
          return jacb_inv * dx;
        }
      }();
    }
    else if (m_alg == JacobianIKAlgorithm::TRANSPOSE) {
      math::VectorXd tmp = jacb * jacb.transpose() * dx;
      double alpha       = dx.dot(tmp) / tmp.dot(tmp);
      dq                 = alpha * jacb.transpose() * dx;
    }
    else {
      CROBOT_ASSERT_ERROR(false, "Unknown Jacobian IK algorithm.");
    }

    m_qpos(qvel_addr) += dq;
  }
  CROBOT_LOG_ERROR(
      "JacobianIK failed to solve after {} iterations.", m_num_iters);

  return false;
}

//==============================================================================
const math::VectorXd& JacobianIK::get_qpos() const {
  return m_qpos;
}

//==============================================================================
void JacobianIK::set_qpos(const math::VectorXd& qpos) {
  m_qpos = qpos;
}
//==============================================================================
void JacobianIK::set_null_space_objective(ik::NullSpaceObj::Ptr obj) {
  m_null_space_obj = obj;
}

//==============================================================================
void JacobianIK::set_lm_bias_damping(double bias_damping) {
  m_lm_bias_damping = bias_damping;
}

//==============================================================================
void JacobianIK::set_lm_step_tolerance(double step_tol) {
  CROBOT_ASSERT_ERROR(
      step_tol >= 0.0, "JacobianIK LM step tolerance must be non-negative.");
  m_lm_step_tol = step_tol;
}
