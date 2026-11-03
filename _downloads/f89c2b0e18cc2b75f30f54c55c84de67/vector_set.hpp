#ifndef VECTOR_SET_HPP
#define VECTOR_SET_HPP

#include "crobot/common/common.hpp"
#include "crobot/math/math.hpp"

namespace crobot {

class VectorSet {
public:
  using Ptr      = std::shared_ptr<VectorSet>;
  using ConstPtr = std::shared_ptr<const VectorSet>;

  VectorSet();
  VectorSet(const std::vector<math::VectorXd>& vectors);

  void add_vector(const math::VectorXd& vector);
  void remove_vector(const math::VectorXd& vector);
  bool is_empty() const;
  int get_size() const;
  std::vector<math::VectorXd> get_vectors() const;
  std::vector<VectorSet::Ptr> get_subsets_down_1() const;

  bool contains(const math::VectorXd& vector, double tor = 1e-6) const;
  bool contains(VectorSet::ConstPtr set, double tor = 1e-6) const;

  VectorSet::Ptr substract(VectorSet::ConstPtr set) const;

  VectorSet::Ptr substract(const math::VectorXd& vec) const;

  VectorSet::Ptr pwise_substract(const math::VectorXd& vec) const;

  math::MatrixXd get_stacked_vectors() const;

  // get a vector that is orthogonal to all vectors in the set
  math::VectorXd get_orthogonal_vector() const;

  // apply the Gram-Schmidt process
  math::VectorXd get_orthogonal_vector_by_GS(const math::VectorXd& vec) const;

  // add orthogonal vectors to the set
  void add_orthogonal_vector();

  bool is_linearly_independent() const;

  bool is_affine_independent() const;

  bool is_identical(const VectorSet::Ptr set) const;

  VectorSet::Ptr replaced(
      const math::VectorXd& old_vec, const math::VectorXd& new_vec) const;

  VectorSet::Ptr replaced(
      VectorSet::ConstPtr old_vecs, VectorSet::ConstPtr new_vecs) const;

  math::VectorXd pop_remove();

  VectorSet::Ptr added(const math::VectorXd& vec) const;

private:
  // get the projection of v onto u
  math::VectorXd get_project(
      const math::VectorXd& v, const math::VectorXd& u) const;

  std::vector<math::VectorXd> m_vectors;
};
} // namespace crobot

#endif // VECTOR_SET_HPP
