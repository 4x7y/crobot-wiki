
#include "crobot/grasping/grasp_quality.hpp"

using namespace crobot;

//==============================================================================
GraspQuality::GraspQuality(
    geometry::MeshPatches::ConstPtr mesh, double mu, double radius_gyration)
  : m_mesh(mesh), m_mu(mu), m_radius_gyration(radius_gyration) {
}

//==============================================================================
double GraspQuality::get_grasp_quality(
    const std::vector<int>& vertex_indices) const {

  std::pair<bool, VectorSet::Ptr> result = is_force_closure(vertex_indices);

  if (!result.first) {
    return -9999.0;
  }

  VectorSet::Ptr V0 = result.second;
  std::vector<int> disk_ind;

  for (size_t i = 0; i < V0->get_vectors().size(); i++) {
    disk_ind.push_back(get_disk_index(vertex_indices, V0->get_vectors()[i]));
    if (disk_ind.back() == -1) {
      return -9999.0;
    }
  }

  while (V0->get_size() < 6) {
    math::VectorXd newV = V0->get_orthogonal_vector();
    auto uk             = get_extreme_point_from_W(vertex_indices, newV);
    V0->add_vector(uk.first);
    disk_ind.push_back(uk.second);
  }
  math::VectorXd tmp = get_b(vertex_indices);

  math::MatrixXd V0_mat = V0->get_stacked_vectors();
  math::VectorXd coeff
      = (V0_mat.transpose() * V0_mat).inverse() * V0_mat.transpose() * tmp;
  CROBOT_ASSERT(
      coeff.minCoeff() > -1e-6, "-w_c is a positive linear combination of V0.");

  V0->add_vector(-tmp);
  CROBOT_ASSERT(V0->is_affine_independent(), "V0 is not affine independent.");

  // create convex hull
  ConvexHull::Ptr convex_hull
      = std::make_shared<ConvexHull>(V0, disk_ind, vertex_indices.size());

  // test if all vectors in V0 are in the same direction
  // bool same_direction = true;
  // for (size_t i = 0; i < V0->get_size(); i++) {
  //   for (size_t j = i + 1; j < V0->get_size(); j++) {
  //     double dot = V0->get_vectors()[i].dot(V0->get_vectors()[j]);
  //     if (dot < 0) {
  //       same_direction = false;
  //       break;
  //     }
  //   }
  // }
  // CROBOT_ASSERT(
  //     !same_direction,
  //     "All vectors in V0 are facing the same direction! Origin is not in the
  //     " "convex hull.");

  // end check

  int min_index;
  double dk = -1;
  math::VectorXd uk;
  double h_uk = 99999;

  while (true) {

    min_index = convex_hull->get_min_index(-1);
    if (min_index == -1) {
      CROBOT_ASSERT(false, "min_index is -1");
    }
    dk = convex_hull->get_dist2origin(min_index);
    uk = convex_hull->get_normal(min_index).normalized();

    h_uk = get_max_projection_from_W(vertex_indices, uk);

    // std::cin.ignore();
    // std::cout << "-----" << std::endl;
    // std::cout << "uk = " << uk.transpose() << std::endl;
    // std::cout << "h_uk = " << h_uk << std::endl;
    // std::cout << "dk = " << dk << std::endl;

    CROBOT_ASSERT(h_uk > 0, "h_uk = %f is not positive.", h_uk);

    if (h_uk - dk < 0.005) {
      return dk;
    }

    auto s_uk = get_extreme_point_from_W(vertex_indices, uk);
    convex_hull->add_vertex(s_uk.first, s_uk.second, min_index);

    if (convex_hull->exceeds(1000)) {
      return dk;
    }

    if (convex_hull->is_empty()) {
      // return -9999.0;
      CROBOT_ASSERT(false, "Convex hull is empty.");
    }
  }

  // should never reach here
  return -9999.0;
}
//==============================================================================
std::pair<bool, VectorSet::Ptr> GraspQuality::is_force_closure(
    const std::vector<int>& vertex_indices) const {
  // step 1
  math::Matrix6X grasp_matrix = get_grasp_matrix(vertex_indices);
  Eigen::FullPivLU<math::Matrix6X> lu(grasp_matrix);
  if (lu.rank() < 6) {
    return std::make_pair(false, std::make_shared<VectorSet>());
  }

  // step 2
  // check if -w_c is in the convex hull of W
  math::Vector6d b = get_b(vertex_indices);

  // step 3
  std::pair<double, VectorSet::Ptr> result = get_dist_to_C(vertex_indices, b);
  if (result.first > 1e-6) {
    return std::make_pair(false, std::make_shared<VectorSet>());
  }

  return std::make_pair(true, result.second);
}

//==============================================================================
math::Vector6d GraspQuality::get_b(
    const std::vector<int>& vertex_indices) const {
  int num_contacts            = vertex_indices.size();
  math::Matrix6X grasp_matrix = get_grasp_matrix(vertex_indices);
  math::Vector6d w_c          = math::Vector6d::Zero();
  for (int i = 0; i < num_contacts; i++) {
    w_c += grasp_matrix.block<6, 1>(0, i * 3);
  }
  w_c /= num_contacts;

  // w_c[3] *= m_radius_gyration;
  // w_c[4] *= m_radius_gyration;
  // w_c[5] *= m_radius_gyration;

  // w_c should be inside the convex hull
  double h_wc = get_max_projection_from_W(vertex_indices, w_c.normalized());
  CROBOT_ASSERT(
      h_wc > w_c.norm(),
      "h_wc = %f is not greater than w_c.norm() = %f.",
      h_wc,
      w_c.norm());

  math::Vector6d b = -w_c;
  if (b.minCoeff() > -1e-6 && b.maxCoeff() < 1e-6) {
    std::cout << "b is zero. Randomizing b." << std::endl;
    CROBOT_ASSERT(false, "b is zero. Randomizing b, there is a problem here");
    b = get_extreme_point_from_W(
            vertex_indices, math::Vector6d::Random().normalized())
            .first;
  }
  return b;
}
//==============================================================================
std::pair<double, VectorSet::Ptr> GraspQuality::get_dist_to_C(
    const std::vector<int>& vertex_indices, const math::Vector6d& b) const {

  // step 1
  math::Vector6d foot     = math::Vector6d::Zero(); // v_0
  math::Vector6d residual = b - foot;
  VectorSet::Ptr A_hat    = std::make_shared<VectorSet>();

  while (true) {

    // step 2
    double h_rk
        = get_max_projection_from_W(vertex_indices, residual.normalized());

    if (residual.norm() < 1e-6 || h_rk < 1e-6) {
      return std::make_pair(residual.norm(), A_hat);
    }

    // step 3
    math::Vector6d s_rk
        = get_extreme_point_from_W(vertex_indices, residual).first;

    // step 4
    A_hat->add_vector(s_rk);

    // step 5
    std::pair<math::Vector6d, VectorSet::Ptr> result = get_support(b, A_hat);

    if (result.second == nullptr) {
      return std::make_pair(9999, nullptr);
    }

    // step 6
    foot  = result.first;
    A_hat = result.second;

    // std::cout << "A_hat: " << std::endl;
    // std::cout << A_hat->get_stacked_vectors() << std::endl;

    CROBOT_ASSERT(A_hat->get_size() <= 6, "A_hat size is greater than 6.");

    // step 7
    residual = b - foot;
  }
}

//==============================================================================
std::pair<math::Vector6d, VectorSet::Ptr> GraspQuality::get_support(
    const math::Vector6d& b, VectorSet::ConstPtr A) const {

  // step 1
  std::vector<math::VectorXd> A_vectors = A->get_vectors();
  math::Vector6d s_rk                   = A_vectors.back();

  while (true) {
    math::Matrix6X M(6, A_vectors.size());
    for (size_t i = 0; i < A_vectors.size(); i++) {
      M.col(i) = A_vectors[i];
    }

    // check if M.transpose() * M is invertible
    math::MatrixXd M_transpose_M = M.transpose() * M;
    Eigen::FullPivLU<math::MatrixXd> lu(M_transpose_M);
    if (lu.rank() < static_cast<Eigen::Index>(A_vectors.size())) {
      return std::make_pair(math::Vector6d::Zero(), nullptr);
    }
    math::VectorXd c = M_transpose_M.inverse() * M.transpose() * b;
    math::Vector6d p = M * c;
    // math::Vector6d residual = b - p;

    // std::cout << "c: " << c.transpose() << std::endl;

    // step 2: partition A based on the sign of c
    VectorSet::Ptr A_pos = std::make_shared<VectorSet>();
    VectorSet::Ptr A_neg = std::make_shared<VectorSet>();

    for (size_t i = 0; i < A_vectors.size(); i++) {
      if (c(i) > 0) {
        A_pos->add_vector(A_vectors[i]);
      }
      else if (c(i) < 0) {
        A_neg->add_vector(A_vectors[i]);
      }
    }

    // step 3
    if (A_neg->is_empty()) {
      return std::make_pair(p, A_pos);
    }
    // else {
    //   std::cout << "out of step 3" << std::endl;
    // }

    // step 4
    if (A_neg->get_size() == 1) {
      std::vector<math::VectorXd> A_vectors_tmp;
      for (auto v : A_vectors) {
        if (v != A_neg->get_vectors()[0])
          A_vectors_tmp.push_back(v);
      }
      A_vectors = A_vectors_tmp;
      continue;
    }

    // std::cout << "out of step 4" << std::endl;

    // step 5: A_neg has more than 1 element
    VectorSet::Ptr A_update = std::make_shared<VectorSet>(A_vectors);
    std::vector<VectorSet::Ptr> subsets = {A_update};
    for (int size = A_vectors.size(); size > 0; size--) {
      std::vector<VectorSet::Ptr> subsets_tmp;
      // get all proper subsets of size reduced by 1
      for (auto subset : subsets) {
        CROBOT_ASSERT(
            subset->get_size() == size, "You messed up with the set size");
        std::vector<VectorSet::Ptr> subsets_down_1
            = subset->get_subsets_down_1();
        subsets_tmp.insert(
            subsets_tmp.end(), subsets_down_1.begin(), subsets_down_1.end());
      }
      subsets = subsets_tmp;

      // iterate over all subsets of size size
      for (auto subset : subsets) {
        // check for subset condition
        if (subset->contains(s_rk, 1e-6) && !subset->contains(A_neg, 1e-6)) {
          // check condition i) all components are positive
          math::Matrix6X N = subset->get_stacked_vectors();
          math::VectorXd mu_i
              = (N.transpose() * N).inverse() * N.transpose() * b;
          if (mu_i.minCoeff() <= 0) {
            continue; // skip to the next subset
          }
          // check condition ii)
          math::Vector6d residual_i = b - N * mu_i;
          VectorSet::Ptr Ai_wo_S    = A_update->substract(subset);
          math::Matrix6X M_S        = Ai_wo_S->get_stacked_vectors();
          math::VectorXd mu_s       = M_S.transpose() * residual_i;
          if (mu_s.maxCoeff() > 0) {
            continue; // skip to the next subset
          }

          // else a stopping condition is met
          return std::make_pair(N * mu_i, subset);
        } // end if
      }   // end for subset
    }     // end for size
    CROBOT_ASSERT(false, "No stopping condition met");
    return std::make_pair(math::Vector6d::Zero(), nullptr);
  }
}

//==============================================================================
std::pair<math::Vector6d, int> GraspQuality::get_extreme_point_from_W(
    std::vector<int> vertex_indices, const math::Vector6d& u) const {

  math::Vector6d u_norm = u.normalized();

  // iterate over all contacts
  math::Vector6d w_max;
  double d_max = -std::numeric_limits<double>::infinity();
  int max_from = -1;

  for (size_t i = 0; i < vertex_indices.size(); i++) {
    math::Matrix63 contact_map = get_contact_map(vertex_indices[i]);
    math::Matrix1X uW          = u_norm.transpose() * contact_map;
    // double a                   = uW(0, 0);
    double b                   = uW(0, 1);
    double c                   = uW(0, 2);

    double denom = std::sqrt(b * b + c * c);
    double alpha = std::atan2(c / denom, b / denom);
    double beta  = alpha + M_PI;

    if (b == 0 && c == 0) {
      alpha = beta = 0; // it doesn't matter
    }

    // double test = std::abs(-b * std::sin(alpha) + c * std::cos(alpha));
    // CROBOT_ASSERT(test < 1e-5, "alpha is not correct. gradient = %f", test);
    // double test2 = std::abs(-b * std::sin(beta) + c * std::cos(beta));
    // CROBOT_ASSERT(test2 < 1e-5, "beta is not correct. gradient = %f", test2);

    math::Vector3d vec1
        = math::Vector3d(1, m_mu * std::cos(alpha), m_mu * std::sin(alpha));
    math::Vector3d vec2
        = math::Vector3d(1, m_mu * std::cos(beta), m_mu * std::sin(beta));

    double d1 = uW * vec1;
    double d2 = uW * vec2;

    if (d1 > d_max) {
      d_max    = d1;
      w_max    = contact_map * vec1;
      max_from = i;
    }

    if (d2 > d_max) {
      d_max    = d2;
      w_max    = contact_map * vec2;
      max_from = i;
    }
  }

  // CROBOT_ASSERT(d_max > 0, "d_max = %f is not positive.", d_max);
  // std::cout << "max_from: " << max_from << std::endl;
  return std::make_pair(w_max, max_from);
}
//==============================================================================
double GraspQuality::get_max_projection_from_W(
    std::vector<int> vertex_indices, const math::Vector6d& u) const {
  return u.normalized().dot(get_extreme_point_from_W(vertex_indices, u).first);
}

//==============================================================================

math::Matrix6X GraspQuality::get_grasp_matrix(
    std::vector<int> vertex_indices) const {
  int num_contacts            = vertex_indices.size();
  math::Matrix6X grasp_matrix = math::Matrix6X::Zero(6, num_contacts * 3);
  for (int i = 0; i < num_contacts; i++) {
    grasp_matrix.block<6, 3>(0, i * 3) = get_contact_map(vertex_indices[i]);
  }
  return grasp_matrix;
}

//==============================================================================
math::Matrix63 GraspQuality::get_contact_map(int vertex_index) const {
  geometry::MeshVertex::ConstPtr vertex = m_mesh->get_vertex(vertex_index);
  math::Vector3d pos          = vertex->get_pos();
  math::Vector3d n = vertex->get_normal_inward().normalized(); // inward normal
  math::Vector3d t1, t2;
  std::tie(t1, t2) = get_tangents(n);
  math::Matrix63 contact_map;
  contact_map.block<3, 1>(0, 0) = n;
  contact_map.block<3, 1>(0, 1) = t1;
  contact_map.block<3, 1>(0, 2) = t2;

  contact_map.block<3, 1>(3, 0) = pos.cross(n) / m_radius_gyration;
  contact_map.block<3, 1>(3, 1) = pos.cross(t1) / m_radius_gyration;
  contact_map.block<3, 1>(3, 2) = pos.cross(t2) / m_radius_gyration;
  return contact_map;
}

//==============================================================================
std::pair<math::Vector3d, math::Vector3d> GraspQuality::get_tangents(
    const math::Vector3d& n) const {
  CROBOT_ASSERT_ERROR(
      std::abs(n.norm() - 1) < 1e-6, "Normal is not normalized.");

  double x = n[0];
  double y = n[1];
  double z = n[2];
  math::Vector3d t1;
  if (std::abs(x) > 0.1) {
    t1 = math::Vector3d(-y, x, 0);
  }
  else if (std::abs(y) > 0.1) {
    t1 = math::Vector3d(0, -z, y);
  }
  else {
    t1 = math::Vector3d(z, 0, -x);
  }
  t1.normalize();
  math::Vector3d t2 = n.cross(t1).normalized();

  // check if t1 and t2 are orthogonal
  CROBOT_ASSERT(
      std::abs(t1.dot(n)) < 1e-6,
      "t1 and n are not orthogonal. dot = %f",
      t1.dot(n));

  return std::make_pair(t1, t2);
}
//==============================================================================

int GraspQuality::get_disk_index(
    const std::vector<int>& vertex_indices, const math::Vector6d& su) const {

  std::vector<double> alpha = {M_PI / 3.0, 2 * M_PI / 3.0, M_PI};

  std::vector<math::Vector3d> vec = {
      math::Vector3d(1, m_mu * std::cos(alpha[0]), m_mu * std::sin(alpha[0])),
      math::Vector3d(1, m_mu * std::cos(alpha[1]), m_mu * std::sin(alpha[1])),
      math::Vector3d(1, m_mu * std::cos(alpha[2]), m_mu * std::sin(alpha[2]))};

  int num_contacts = vertex_indices.size();
  for (int i = 0; i < num_contacts; i++) {
    int vertex                 = vertex_indices[i];
    math::Matrix63 contact_map = get_contact_map(vertex);
    std::vector<math::VectorXd> wrenches;
    for (int j = 0; j < 3; j++) {
      wrenches.push_back(contact_map * vec[j]);
    }

    VectorSet::Ptr W = std::make_shared<VectorSet>(wrenches);

    if (W->contains(su, 1e-6))
      return i;

    W->add_vector(su);
    if (!W->is_linearly_independent())
      return i;
  }

  // CROBOT_ASSERT(false, "No disk index found. should not happen!");
  return -1;
}
