#ifndef GRASP_SOLVER_HPP
#define GRASP_SOLVER_HPP
#include <queue>

#include "crobot/common/common.hpp"
#include "crobot/grasping/grasp_quality.hpp"
#include "crobot/geometry/mesh_patches.hpp"
#include "crobot/patch_tree/patch_tree.hpp"
#include "crobot/math/math.hpp"

namespace crobot {

class GraspSolver {
public:
  using Ptr      = std::shared_ptr<GraspSolver>;
  using ConstPtr = std::shared_ptr<const GraspSolver>;

  // Callback function type for visualization
  // Parameters: vertex positions to visualize
  using VisualizationCallback = std::function<void(
      const std::vector<math::Vector3d>& vertex_positions)>;

  GraspSolver(
      geometry::MeshPatches::ConstPtr mesh,
      GraspQuality::ConstPtr quality,
      crobot::patch_tree::PatchTree::ConstPtr patch_tree);

  GraspSolver(
      geometry::MeshPatches::ConstPtr mesh,
      GraspQuality::ConstPtr quality,
      crobot::patch_tree::PatchTree::ConstPtr patch_tree,
      VisualizationCallback visualization_callback);

  struct Compare
  {
    bool operator()(
        const std::pair<double, std::vector<int>>& p1,
        const std::pair<double, std::vector<int>>& p2) {
      // Compare based on the key (double)
      return p1.first > p2.first;
    }
  };

  // use patch tree
  std::pair<double, std::vector<int>> solve_grasp(int num_fingers);

  // use patch tree
  std::pair<double, std::vector<int>> solve_grasp_BFS(int num_fingers);

  // try all possible combinations of patch_labels
  std::pair<double, std::vector<int>> solve_grasp_naive(int num_fingers);

  // local optimization within patch_labels
  // return the quality and the vertex indices
  std::pair<double, std::vector<int>> solve_grasp(
      const std::vector<int>& patch_labels,
      int max_round     = 100,
      double quality_lb = 0.0);

  double get_center_grasp_quality(const std::vector<int>& patch_labels) const;

private:
  std::vector<int> generate_random_permutation(int n) const;

  std::vector<int> permu(const std::vector<int>& v) const;

  std::vector<std::vector<int>> generateSubsets(
      const std::vector<int>& set, int k, int index) const;

  std::vector<std::vector<int>> generate_permutations_with_duplicates(
      const std::vector<int>& set, int n) const;

  void update_sim(const std::vector<int>& vertex_indices) const;

  void save_yaml(
      const std::string& filename,
      std::priority_queue<
          std::pair<double, std::vector<int>>,
          std::vector<std::pair<double, std::vector<int>>>,
          GraspSolver::Compare> pq) const;

  geometry::MeshPatches::ConstPtr m_mesh;
  GraspQuality::ConstPtr m_quality;
  crobot::patch_tree::PatchTree::ConstPtr m_patch_tree;

  int m_num_finger_adj;

  // for visualization
  VisualizationCallback m_visualization_callback;
};
} // namespace crobot

#endif // GRASP_SOLVER_HPP
