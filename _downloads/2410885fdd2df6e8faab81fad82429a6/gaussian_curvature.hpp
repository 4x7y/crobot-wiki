#pragma once

#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <open3d/geometry/TriangleMesh.h>

namespace crobot::geometry {

/**
 * @brief Gaussian curvature computation utilities
 */
class GaussianCurvature {
public:
  /**
   * @brief Compute Gaussian curvature using angle deficit method
   * @param mesh Input triangle mesh
   * @return Vector of Gaussian curvature values per vertex
   */
  static Eigen::VectorXd compute_angle_deficit(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);

  /**
   * @brief Compute smoothed Gaussian curvature using k-ring neighborhood
   * @param mesh Input triangle mesh
   * @param ring_k Size of the k-ring neighborhood (default: 2)
   * @return Vector of smoothed Gaussian curvature values per vertex
   */
  static Eigen::VectorXd compute_kring_smoothed(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
      int ring_k = 2);

private:
  /**
   * @brief Build vertex adjacency list from mesh
   */
  static std::vector<std::unordered_set<int>> build_vertex_adjacency(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);

  /**
   * @brief Get k-ring neighborhood of a vertex
   */
  static std::unordered_set<int> get_kring_neighbors(
      const std::vector<std::unordered_set<int>>& adjacency,
      int start_vertex,
      int k);

  /**
   * @brief Compute angle between two vectors
   */
  static double compute_angle(
      const Eigen::Vector3d& u, const Eigen::Vector3d& v);

  /**
   * @brief Detect boundary vertices
   */
  static std::unordered_set<int> detect_boundary_vertices(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);
};

} // namespace crobot::geometry
