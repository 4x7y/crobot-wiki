#pragma once

#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <open3d/geometry/Geometry.h>
#include <open3d/geometry/TriangleMesh.h>

namespace crobot::geometry {

constexpr int MIN_PATCH_VERTICES_NUM  = 200;
constexpr double MAX_TORQUE_VARIATION = 0.02;

/**
 * @brief Utility functions for mesh processing
 */
class MeshUtils {
public:
  static std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
  get_vertices_and_normals(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);

  // Inherit the source normal field; compute it only when absent. By default
  // indices are preserved. Compact export retains only referenced vertices.
  static std::shared_ptr<open3d::geometry::TriangleMesh> extract_triangles(
      const open3d::geometry::TriangleMesh& mesh,
      const std::vector<int>& triangle_ids,
      bool compact_vertices = false);

  // Eight corners of the axis-aligned normal-space box containing all
  // inherited vertex normals referenced by these faces. The corners need
  // not be unit vectors: their convex hull conservatively bounds the normal
  // samples, and their positive hull contains normalized interpolations.
  // Degenerate boxes retain eight corners; empty face selection returns {}.
  static std::vector<Eigen::Vector3d> enclosing_normal_directions(
      const open3d::geometry::TriangleMesh& mesh,
      const std::vector<int>& triangle_ids);

  /**
   * @brief Expand vertex IDs by radius using spatial search
   */
  static std::vector<int> expand_vertex_ids_by_radius(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
      const std::vector<int>& seed_vertex_ids,
      double radius);

  /**
   * @brief Filter dense red vertices based on neighbor count
   */
  static std::vector<int> filter_dense_red_vertices(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
      const std::vector<int>& red_vertex_ids,
      double radius = 0.01,
      int k         = 10);

  /**
   * @brief Compute solid angle for a triangle defined by three points
   */
  static double compute_solid_angle(
      const Eigen::Vector3d& p1,
      const Eigen::Vector3d& p2,
      const Eigen::Vector3d& p3);

  /**
   * @brief Check if a ray intersects with a triangle
   */
  static bool ray_intersects_triangle(
      const Eigen::Vector3d& dir,
      const Eigen::Vector3d& v0,
      const Eigen::Vector3d& v1,
      const Eigen::Vector3d& v2);

  /**
   * @brief Generate subdivided mesh
   */
  static std::shared_ptr<open3d::geometry::TriangleMesh>
  generate_subdivided_mesh(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
      int number_of_iterations = 2);

  /**
   * @brief Create arrow mesh for visualization
   */
  static std::shared_ptr<open3d::geometry::TriangleMesh> create_arrow(
      const Eigen::Vector3d& end,
      const Eigen::Vector3d& origin = Eigen::Vector3d::Zero(),
      double scale                  = 1.0);

private:
  static std::pair<Eigen::Matrix3d, Eigen::Matrix3d>
  calculate_zy_rotation_for_arrow(const Eigen::Vector3d& vec);
};

} // namespace crobot::geometry
