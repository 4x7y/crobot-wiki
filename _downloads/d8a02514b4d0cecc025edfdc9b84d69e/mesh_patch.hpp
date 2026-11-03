#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

// Forward declarations
namespace open3d {
namespace geometry {
class TriangleMesh;
}
} // namespace open3d

namespace crobot::patch_tree {

/**
 * @brief A mesh region undergoing normal-based subdivision. MeshPatches is
 * the contact-planning view of saved partitions, backed by PatchMeshData.
 */
class MeshPatch {
public:
  static constexpr double MAX_TOTAL_CURVATURE_PER_PATCH = 4.0 * M_PI / 100.0;

  MeshPatch(
      std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
      const std::vector<Eigen::Vector3d>& icosahedron_triangle);

  /**
   * @brief Add a triangle to this patch
   */
  void add_triangle(
      int tid, const Eigen::Vector3d& normal, double solid_angle = -1.0);

  // Extract surface geometry without building a Gauss-map visualization.
  // Returns nullptr for an empty patch; preserves source vertex indices.
  std::shared_ptr<open3d::geometry::TriangleMesh> get_surface_mesh() const;

  /**
   * @brief Convert patch to mesh representation
   */
  std::pair<
      std::shared_ptr<open3d::geometry::TriangleMesh>,
      std::shared_ptr<open3d::geometry::TriangleMesh>>
  convert_to_mesh() const;

  /**
   * @brief Decompose patch into smaller sub-patches
   */
  std::vector<std::shared_ptr<MeshPatch>> decompose();

  /**
   * @brief Check if total curvature exceeds threshold
   */
  bool is_total_curvature_exceeding_threshold() const;

  /**
   * @brief Check if a vector intersects with this patch
   */
  bool is_intersected_with_vector(const Eigen::Vector3d& vector) const;

  int get_num_mesh_triangles() const {
    return static_cast<int>(mesh_tids_.size());
  }
  // Source face IDs, for inspecting partition coverage without extracting
  // meshes.
  const std::vector<int>& get_triangle_ids() const {
    return mesh_tids_;
  }
  double get_total_curvature() const {
    return total_curvature_;
  }

private:
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
  std::vector<Eigen::Vector3d> icosahedron_triangle_;
  std::vector<int> mesh_tids_;
  std::vector<Eigen::Vector3d> mesh_triangle_normals_;
  std::vector<double> solid_angles_;
  double total_curvature_;
};

} // namespace crobot::patch_tree
