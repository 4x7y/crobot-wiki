
#include "crobot/grasping/vector_set.hpp"

using namespace crobot;

//==============================================================================
VectorSet::VectorSet() {
  m_vectors.clear();
}

//==============================================================================
VectorSet::VectorSet(const std::vector<math::VectorXd>& vectors)
  : m_vectors(vectors) {
}

//==============================================================================
void VectorSet::add_vector(const math::VectorXd& vector) {
  // check if vector already exists
  for (size_t i = 0; i < m_vectors.size(); i++) {
    if (contains(vector, 1e-10)) {
      CROBOT_ASSERT(false, "Vector already exists");
      // switch vector to the last element
      // math::VectorXd temp = m_vectors[i];
      // m_vectors[i]        = m_vectors.back();
      // m_vectors.back()    = temp;
      return;
    }
  }
  m_vectors.push_back(vector);
}

//==============================================================================
bool VectorSet::is_empty() const {
  return m_vectors.empty();
}
//==============================================================================
int VectorSet::get_size() const {
  return m_vectors.size();
}
//==============================================================================
std::vector<math::VectorXd> VectorSet::get_vectors() const {
  return m_vectors;
}
//==============================================================================
std::vector<VectorSet::Ptr> VectorSet::get_subsets_down_1() const {
  std::vector<VectorSet::Ptr> subsets;
  if (m_vectors.empty()) {
    CROBOT_ASSERT(false, "Vector set is empty");
    return subsets;
  }

  for (size_t i = 0; i < m_vectors.size(); i++) {
    std::vector<math::VectorXd> subset_vectors = m_vectors;
    subset_vectors.erase(subset_vectors.begin() + i);
    subsets.push_back(std::make_shared<VectorSet>(subset_vectors));
  }

  return subsets;
}
//==============================================================================
bool VectorSet::contains(const math::VectorXd& vector, double tor) const {
  for (size_t i = 0; i < m_vectors.size(); i++) {
    if ((m_vectors[i] - vector).maxCoeff() < tor
        && (m_vectors[i] - vector).minCoeff() > -tor) {
      return true;
    }
  }
  return false;
}

//==============================================================================
bool VectorSet::contains(VectorSet::ConstPtr set, double tor) const {
  for (size_t i = 0; i < set->get_vectors().size(); i++) {
    if (!contains(set->get_vectors()[i], tor)) {
      return false;
    }
  }
  return true;
}

//==============================================================================
VectorSet::Ptr VectorSet::substract(VectorSet::ConstPtr set) const {
  std::vector<math::VectorXd> vectors;
  for (size_t i = 0; i < m_vectors.size(); i++) {
    if (!set->contains(m_vectors[i], 1e-6)) {
      vectors.push_back(m_vectors[i]);
    }
  }
  return std::make_shared<VectorSet>(vectors);
}

//==============================================================================
VectorSet::Ptr VectorSet::substract(const math::VectorXd& vec) const {
  std::vector<math::VectorXd> vectors;
  for (size_t i = 0; i < m_vectors.size(); i++) {
    if (m_vectors[i] != vec) {
      vectors.push_back(m_vectors[i]);
    }
  }
  return std::make_shared<VectorSet>(vectors);
}

//==============================================================================
void VectorSet::remove_vector(const math::VectorXd& v) {
  for (size_t i = 0; i < m_vectors.size(); i++) {
    if (m_vectors[i] == v) {
      m_vectors.erase(m_vectors.begin() + i);
      return;
    }
  }
}

//==============================================================================
VectorSet::Ptr VectorSet::pwise_substract(const math::VectorXd& vec) const {
  std::vector<math::VectorXd> vectors;
  for (size_t i = 0; i < m_vectors.size(); i++) {
    if (m_vectors[i] != vec) {
      vectors.push_back(m_vectors[i] - vec);
      CROBOT_ASSERT(vectors.back().norm() > 1e-10, "Error in pwise_substract");
    }
  }

  // std::cout << std::endl;
  // std::cout << "vec = " << vec.transpose() << std::endl;
  // std::cout << "---- before sub ----" << std::endl;
  // for (size_t i = 0; i < m_vectors.size(); i++) {
  //   std::cout << m_vectors[i].transpose() << std::endl;
  // }
  // std::cout << "---- after sub ----" << std::endl;
  // for (size_t i = 0; i < vectors.size(); i++) {
  //   std::cout << vectors[i].transpose() << std::endl;
  // }
  // std::cout << std::endl;

  // CROBOT_ASSERT(
  //     vectors.size() == m_vectors.size() - 1,
  //     "Error in pwise_substract, vectors.size = %zu, m_vectors.size = %zu",
  //     vectors.size(),
  //     m_vectors.size());

  return std::make_shared<VectorSet>(vectors);
}

//==============================================================================
math::MatrixXd VectorSet::get_stacked_vectors() const {
  CROBOT_ASSERT(!is_empty(), "Vector set is empty");
  math::MatrixXd mat(m_vectors[0].size(), m_vectors.size());
  for (size_t i = 0; i < m_vectors.size(); i++) {
    mat.col(i) = m_vectors[i];
  }
  return mat;
}

//==============================================================================
math::VectorXd VectorSet::get_orthogonal_vector() const {
  CROBOT_ASSERT(!is_empty(), "Vector set is empty");
  CROBOT_ASSERT(
      m_vectors[0].size() > static_cast<Eigen::Index>(m_vectors.size()), "Too many vectors in the set");

  CROBOT_ASSERT(
      is_linearly_independent(), "[GE] Vectors are not linearly independent");

  // find an orthogonal vector to all vectors in the set
  math::MatrixXd mat = get_stacked_vectors();
  // solve a non-zero vector that satisfies the equation
  // mat^T * v = 0
  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(mat.transpose());
  Eigen::VectorXd v = lu_decomp.kernel().col(0);

  CROBOT_ASSERT(
      v.norm() > 1e-10, "Error in get_orthogonal_vector, norm = %f", v.norm());

  // test
  // for (size_t i = 0; i < m_vectors.size(); i++) {
  //   CROBOT_ASSERT(
  //       std::abs(v.dot(m_vectors[i])) < 1e-6,
  //       "Error in get_orthogonal_vector, ret = %f",
  //       v.dot(m_vectors[i]));
  // }

  return v.normalized();
}

//==============================================================================
math::VectorXd VectorSet::get_orthogonal_vector_by_GS(
    const math::VectorXd& vec) const {

  CROBOT_ASSERT(
      is_linearly_independent(), "[GS] Vectors are not linearly independent");

  // apply the Gram-Schmidt process
  std::vector<math::VectorXd> u;
  for (size_t i = 0; i < m_vectors.size(); i++) {
    math::VectorXd u_i = m_vectors[i];
    for (size_t j = 0; j < i; j++) {
      u_i -= get_project(m_vectors[i], u[j]);
    }
    CROBOT_ASSERT(
        u_i.norm() > 1e-6,
        "Error in get_orthogonal_vector_by_GS, u_i.norm = %f",
        u_i.norm());
    u.push_back(u_i);
  }

  math::VectorXd v(vec);

  for (size_t i = 0; i < u.size(); i++) {
    v -= get_project(v, u[i]);
  }

  // std::cout << "v = " << v.transpose() << std::endl;
  // std::cout << "v.norm = " << v.norm() << std::endl;
  // std::cout << "v.normalized = " << v.normalized().transpose() << std::endl;
  // std::cout << "v.normalized.norm = " << v.normalized().dot(v.normalized())
  //           << std::endl;

  CROBOT_ASSERT(
      v.norm() > 1e-10,
      "Error in get_orthogonal_vector_by_GS, v.norm = %f",
      v.norm());

  // test
  // for (size_t i = 0; i < m_vectors.size(); i++) {
  //   if (std::abs(v.dot(m_vectors[i])) > 1e-6) {
  //     math::VectorXd v_ = get_orthogonal_vector();
  //     std::cout << "\n\nGS error, result by guassian elimi = "
  //               << v_.dot(m_vectors[i]) << std::endl;
  //     std::cout << "result by GS = " << v << std::endl;
  //     CROBOT_ASSERT(
  //         false,
  //         "Error in get_orthogonal_vector_by_GS, ret = %f",
  //         v.dot(m_vectors[i]));
  //   }
  // }

  return v;
}

//==============================================================================
void VectorSet::add_orthogonal_vector() {
  m_vectors.push_back(get_orthogonal_vector());
}

//==============================================================================
math::VectorXd VectorSet::get_project(
    const math::VectorXd& v, const math::VectorXd& u) const {
  return (v.dot(u) / u.dot(u)) * u;
}

//==============================================================================
bool VectorSet::is_linearly_independent() const {
  if (static_cast<Eigen::Index>(m_vectors.size()) > m_vectors[0].size()) {
    return false;
  }

  math::MatrixXd mat = get_stacked_vectors();

  // calculate eigen values
  Eigen::EigenSolver<math::MatrixXd> eigensolver(mat.transpose() * mat);
  math::VectorXd eigenvalues = eigensolver.eigenvalues().real();

  for (int i = 0; i < eigenvalues.size(); i++) {
    if (std::abs(eigenvalues[i]) < 1e-10) {
      return false;
    }
  }

  return true;
}

//==============================================================================
bool VectorSet::is_affine_independent() const {
  if (static_cast<Eigen::Index>(m_vectors.size()) > m_vectors[0].size() + 1) {
    return false;
  }

  return pwise_substract(m_vectors[0])->is_linearly_independent();
}

//==============================================================================
bool VectorSet::is_identical(const VectorSet::Ptr set) const {
  if (static_cast<int>(m_vectors.size()) != set->get_size()) {
    return false;
  }

  for (size_t i = 0; i < m_vectors.size(); i++) {
    if (!set->contains(m_vectors[i], 1e-6)) {
      return false;
    }
  }

  return true;
}

//==============================================================================
VectorSet::Ptr VectorSet::replaced(
    const math::VectorXd& old_vec, const math::VectorXd& new_vec) const {
  std::vector<math::VectorXd> vectors = m_vectors;
  for (size_t i = 0; i < vectors.size(); i++) {
    if (vectors[i] == old_vec) {
      vectors[i] = new_vec;
      break;
    }
  }
  return std::make_shared<VectorSet>(vectors);
}

//==============================================================================
VectorSet::Ptr VectorSet::replaced(
    VectorSet::ConstPtr old_vecs, VectorSet::ConstPtr new_vecs) const {
  CROBOT_ASSERT(
      old_vecs->get_size() == new_vecs->get_size(),
      "old_vecs->get_size() = %d, new_vecs->get_size() = %d",
      old_vecs->get_size(),
      new_vecs->get_size());

  std::vector<math::VectorXd> vectors = m_vectors;
  for (size_t i = 0; i < static_cast<size_t>(old_vecs->get_size()); i++) {
    for (size_t j = 0; j < vectors.size(); j++) {
      if (vectors[j] == old_vecs->get_vectors()[i]) {
        vectors[j] = new_vecs->get_vectors()[i];
        break;
      }
    }
  }
  return std::make_shared<VectorSet>(vectors);
}

//==============================================================================
math::VectorXd VectorSet::pop_remove() {
  CROBOT_ASSERT(!is_empty(), "Vector set is empty");
  math::VectorXd v = m_vectors.back();
  m_vectors.pop_back();
  return v;
}

//==============================================================================
VectorSet::Ptr VectorSet::added(const math::VectorXd& vec) const {
  std::vector<math::VectorXd> vectors = m_vectors;
  vectors.push_back(vec);
  return std::make_shared<VectorSet>(vectors);
}
