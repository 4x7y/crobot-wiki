#ifndef GRASP_QUALITY_HPP
#define GRASP_QUALITY_HPP

#include "crobot/common/common.hpp"
#include "crobot/math/math.hpp"

#include "crobot/grasping/convex_hull.hpp"
#include "crobot/geometry/mesh_patches.hpp"
#include "crobot/grasping/vector_set.hpp"

namespace crobot {

class GraspQuality {
public:
  using Ptr      = std::shared_ptr<GraspQuality>;
  using ConstPtr = std::shared_ptr<const GraspQuality>;

  GraspQuality(geometry::MeshPatches::ConstPtr mesh, double mu, double radius_gyration);

  std::pair<bool, VectorSet::Ptr> is_force_closure(
      const std::vector<int>& vertex_indices) const;

  double get_grasp_quality(const std::vector<int>& vertex_indices) const;

  std::pair<double, VectorSet::Ptr> get_dist_to_C(
      const std::vector<int>& vertex_indices, const math::Vector6d& b) const;

  // subroutine for computing foot and A_hat
  std::pair<math::Vector6d, VectorSet::Ptr> get_support(
      const math::Vector6d& b, VectorSet::ConstPtr A) const;

  math::Matrix63 get_contact_map(int vertex_index) const;

  math::Matrix6X get_grasp_matrix(std::vector<int> vertex_indices) const;

  // s(u) in appendix B of part 2
  std::pair<math::Vector6d, int> get_extreme_point_from_W(
      std::vector<int> vertex_indices, const math::Vector6d& u) const;

  // h(u)
  double get_max_projection_from_W(
      std::vector<int> vertex_indices, const math::Vector6d& u) const;

private:
  std::pair<math::Vector3d, math::Vector3d> get_tangents(
      const math::Vector3d& n) const;

  int get_disk_index(
      const std::vector<int>& vertex_indices, const math::Vector6d& su) const;

  math::Vector6d get_b(const std::vector<int>& vertex_indices) const;

  geometry::MeshPatches::ConstPtr m_mesh;
  double m_mu;
  double m_radius_gyration;
};
} // namespace crobot

#endif // GRASP_QUALITY_HPP
