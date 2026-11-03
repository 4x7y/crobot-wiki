#pragma once

#include <map>
#include <memory>
#include <vector>

#include "crobot/patch_tree/mesh_patch.hpp"
#include "crobot/patch_tree/monotone_decomposition.hpp"
#include "crobot/patch_tree/patch_tree.hpp"

namespace open3d {
namespace geometry {
class TriangleMesh;
}
} // namespace open3d

namespace crobot::patch_tree {

struct PatchGenerationOptions
{
  double max_total_curvature = MeshPatch::MAX_TOTAL_CURVATURE_PER_PATCH;
  // Zero disables torque subdivision (the legacy normal-tree interface).
  double max_torque_variation = 0.02;
  // Conservative pairwise angular bound on the inherited vertex normals.
  double max_normal_angle = 20.0 * M_PI / 180.0;
  // Root depth is zero; the three monotone cuts precede spatial/normal/torque.
  int max_depth           = 32;
  bool split_by_curvature = true;
  MonotoneOptions monotone;
  // Connected spatial subdivisions within each original curvature region.
  // Zero disables these; the accepted monotone hierarchy is still retained.
  std::size_t max_coarse_faces = 512;
  // Applied by MeshDecomposer::generate_patch_tree before returning the final
  // hierarchy. MeshPatchTree itself never changes its input triangulation.
  int max_refinement_rounds      = 12;
  std::size_t max_prepared_faces = 500000;
};

/**
 * Deterministic mesh subdivision. add_triangle() collects source face IDs;
 * finalize()/inspection constructs the hierarchy in sorted face order.
 * Every non-root node is edge-connected. Geometry and normals are shared.
 * The mesh must remain unchanged for the lifetime of this generator.
 */
class MeshPatchTree {
public:
  struct Node
  {
    int id;
    int parent_id;
    std::shared_ptr<MeshPatch> patch;
    std::vector<int> children_ids;
    PatchTree::NodeData data;

    Node(int id, int parent_id, std::shared_ptr<MeshPatch> patch)
      : id(id), parent_id(parent_id), patch(std::move(patch)) {
    }
  };

  // Compatibility: normal subdivision only, with the new coverage guarantees.
  MeshPatchTree(
      std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
      const Eigen::Vector3d& center);
  MeshPatchTree(
      std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
      const Eigen::Vector3d& center,
      const PatchGenerationOptions& options,
      std::shared_ptr<const MonotoneDecomposition> monotone = {},
      std::vector<int> analysis_face_ids                    = {});

  void add_triangle(int tid);
  void finalize() const;

  // Compact export preserves the prepared normal field. Visualization is
  // opt-in; the second mesh in each pair is null when it is disabled.
  std::vector<std::pair<
      std::shared_ptr<open3d::geometry::TriangleMesh>,
      std::shared_ptr<open3d::geometry::TriangleMesh>>>
  gather_patches(
      bool include_visualization = false, bool compact_vertices = true);

  double compute_total_curvature() const;
  const std::map<int, Node>& get_nodes() const;
  PatchTree::Ptr get_planning_tree(
      const std::vector<std::vector<int>>& face_source_ids = {}) const;
  std::vector<int> get_face_leaf_ids() const;
  void show() const;

private:
  struct Bounds
  {
    double curvature              = 0;
    double normal_angle           = 0;
    double torque_span            = 0;
    Eigen::Matrix3d torque_axes   = Eigen::Matrix3d::Identity();
    Eigen::Vector3d torque_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_extent = Eigen::Vector3d::Zero();
  };

  void ensure_built() const;
  void prepare_geometry();
  Bounds measure(const std::vector<int>& ids) const;
  std::vector<std::vector<int>> connected_groups(
      const std::vector<int>& ids, const std::vector<int>& labels) const;
  std::vector<int> spatial_bisection(const std::vector<int>& ids) const;
  std::vector<std::vector<int>> connected_bisection(
      const std::vector<int>& ids) const;
  int add_node(
      int parent,
      const std::vector<int>& ids,
      int type,
      const std::vector<Eigen::Vector3d>& triangle) const;
  void refine_normal(
      int id, int depth, const std::vector<Eigen::Vector3d>& triangle) const;
  void refine_torque(int id, int depth) const;
  void set_leaf_status(int id, int depth) const;
  void show_recursive(int id, int depth) const;

  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
  Eigen::Vector3d center_;
  PatchGenerationOptions options_;
  std::shared_ptr<const MonotoneDecomposition> monotone_;
  std::vector<int> analysis_face_ids_;
  std::vector<int> input_ids_;
  std::vector<bool> selected_;
  std::vector<std::vector<int>> face_neighbors_;
  std::vector<std::pair<int, int>> face_edges_;
  std::vector<Eigen::Vector3d> face_normals_, face_centers_;
  std::vector<double> face_curvatures_;
  mutable std::map<int, Node> nodes_;
  mutable PatchTree::Ptr planning_tree_;
  mutable bool dirty_ = true;
};

} // namespace crobot::patch_tree
