#pragma once

#include <string>
#include <vector>

#include "crobot/geometry/mesh_preprocessor.hpp"
#include "crobot/patch_tree/mesh_patch_tree.hpp"

// Forward declarations
namespace open3d {
namespace geometry {
class TriangleMesh;
}
} // namespace open3d

namespace crobot::patch_tree {

struct GeneratedPatchTree
{
  // Bounds refer to this origin and the prepared mesh's coordinate frame.
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  PatchGenerationOptions options;
  geometry::MeshPreparationOptions preparation_options;
  geometry::PreparedMesh surface;
  // Analysis is computed once before normal/torque mesh refinement.
  std::shared_ptr<const open3d::geometry::TriangleMesh> analysis_mesh;
  std::shared_ptr<const MonotoneDecomposition> monotone;
  // Current prepared face -> original analysis face (also after refinement).
  std::vector<int> face_analysis_ids;
  PatchTree::Ptr tree;
  // Prepared triangle index -> terminal planning node ID.
  std::vector<int> face_leaf_ids;
  // True if hierarchy or mesh refinement stopped at a configured limit
  // while some leaves still have explicitly unresolved bounds.
  bool refinement_limit_reached = false;
  // Empty on success; explains a budget or interpolation stop when bounds
  // remain unresolved. Inspect each leaf's bound statuses before planning.
  std::string refinement_stop_reason;
};

/**
 * @brief Main class for mesh decomposition operations
 */
class MeshDecomposer {
public:
  // Build the prepared shared surface and the existing PatchTree planning
  // hierarchy, including per-level adjacency, source provenance, and bounds.
  // Refinement splits triangles without moving the original surface. Bounds
  // are evaluated on prepared vertex normals and torques; unresolved limits
  // remain explicit in leaf statuses. Keep the returned surface unchanged
  // while using its hierarchy. No collision or grasp feasibility is implied.
  static GeneratedPatchTree generate_patch_tree(
      std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
      const Eigen::Vector3d& origin,
      const PatchGenerationOptions& options               = {},
      const geometry::MeshPreparationOptions& preparation = {});

  /**
   * @brief Export the leaves of generate_patch_tree as compact meshes.
   * All prepared faces are retained, including small surface regions.
   */
  static void decompose_mesh(
      std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
      const Eigen::Vector3d& origin,
      std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>>&
          submeshes_torque,
      bool save_mesh                    = false,
      const std::string& base_directory = "",
      double max_torque_variation = 0.02);

  /**
   * @brief Generate submeshes based on Gaussian curvature
   */
  static std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>>
  generate_submeshes(std::shared_ptr<open3d::geometry::TriangleMesh> mesh);

};

} // namespace crobot::patch_tree
