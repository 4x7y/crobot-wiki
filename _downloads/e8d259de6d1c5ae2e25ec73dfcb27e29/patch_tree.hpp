#ifndef PATCH_TREE_HPP
#define PATCH_TREE_HPP

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "crobot/patch_tree/patch_node.hpp"

namespace crobot::geometry {
class PatchMeshData;
}

namespace crobot::patch_tree {

class PatchTree {
public:
  using Ptr      = std::shared_ptr<PatchTree>;
  using ConstPtr = std::shared_ptr<const PatchTree>;

  /**
   * A patch in a level graph. IDs refer to the original PatchNodes.
   * parent_id and child_ids link adjacent level graphs, while neighbor_ids
   * are undirected surface adjacency within this level. A terminal patch
   * carried into the next level has itself as its only child there.
   * Links may skip intermediate tree nodes whose surface partition did not
   * change; use these links, not NodeData.parent_id, between graph levels.
   */
  struct LevelPatch
  {
    int parent_id = -1;
    std::vector<int> child_ids;
    std::vector<int> neighbor_ids;
  };

  using LevelGraph = std::map<int, LevelPatch>;
  using PatchEdge  = std::pair<int, int>;

  // A terminal patch can remain above a requested bound when a single face
  // cannot be split further or a configured refinement limit was reached.
  enum class BoundStatus
  {
    NotEvaluated,
    Satisfied,
    Indivisible,
    RefinementLimit
  };

  enum class Stage
  {
    Unspecified,
    Object,
    Component,
    Monotone,
    DetailGrouping,
    CurvatureRegion,
    Spatial,
    Normal,
    Torque
  };

  /**
   * In-memory node description. The vector index is the node ID; root 0 has
   * parent_id -1. Type 1 is a whole-object, component, or coarse spatial node;
   * type 2 is a normal-domain patch, and type 3 a torque-domain patch.
   * vertices contains a conservative normal/torque bounding polytope used by
   * legacy grasp tests, never the vertices of the supporting surface mesh.
   * These descriptor points need not be unit vectors or lie on the surface.
   *
   * face_ids index a separately owned prepared mesh. Children must partition
   * their parent's IDs exactly. source_face_ids optionally records membership
   * in the original mesh; overlaps between children are allowed when an
   * original face was subdivided. Both lists are returned in sorted order.
   */
  struct NodeData
  {
    int parent_id = -1;
    int type      = 1;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<int> face_ids;
    std::vector<int> source_face_ids;
    BoundStatus curvature_status = BoundStatus::NotEvaluated;
    BoundStatus normal_status    = BoundStatus::NotEvaluated;
    BoundStatus torque_status    = BoundStatus::NotEvaluated;
    double total_curvature       = 0.0;
    double max_normal_angle      = 0.0; // radians
    double torque_span           = 0.0;
    Stage stage                  = Stage::Unspecified;
    // Approximate class; -1 denotes a mixed/coarse region. Per-face uncertainty
    // remains separate from assigned parent class in GeneratedPatchTree.
    int curvature_class            = -1;
    int monotone_region_id         = -1;
    double uncertain_area_fraction = 0.0;
  };

  PatchTree(const std::string& folder, int num, double scale);

  /**
   * Construct a live planning hierarchy without saved YAML/CSV files. Records
   * must form one rooted tree, with nonempty support except for an empty root.
   * The root's face IDs define the represented surface. Type 2 descriptors
   * need at least three finite points and type 3 at least four. Adjacency must
   * connect leaves sharing surface edges; duplicate/self edges are ignored.
   * Every non-root internal patch must be connected in the supplied leaf
   * adjacency graph. The generator is responsible for each leaf's geometric
   * connectivity and for matching face IDs to its prepared surface.
   *
   * Level graphs are built automatically. Invalid data throws
   * std::invalid_argument. No Open3D surface ownership is introduced here.
   */
  PatchTree(
      std::vector<NodeData> nodes, const std::vector<PatchEdge>& leaf_edges);

  PatchNode::ConstPtr get_root() const;

  PatchNode::ConstPtr get_node(int node_id) const;

  int get_num_nodes() const;

  // Saved trees do not contain per-node face membership or bound diagnostics.
  bool has_face_membership() const;
  const NodeData& get_node_data(int node_id) const;
  const std::vector<int>& get_face_ids(int node_id) const;

  /**
   * Build level graphs from vertices.csv, faces.csv and face_labels.csv in
   * the loaded partitions. Only leaves with labeled faces are included.
   * Faces labeled -1 are excluded; they do not bridge gaps between patches.
   * Shared edges use exactly equal endpoint coordinates, including across
   * partitions (the saved files have no global vertex IDs). Point contact
   * alone does not create an edge. Coincident surfaces are therefore welded;
   * seams with unequal coordinates are not. Use the explicit overload when
   * the original mesh topology is available or a different rule is needed.
   * Existing grasp-search methods do not require this optional step.
   *
   * @code
   * PatchTree tree(folder, num_partitions, scale);
   * tree.build_level_graphs();
   * for (const auto& [id, patch] : tree.get_level_graph(1)) {
   *   // patch.neighbor_ids: moves at this resolution
   *   // patch.child_ids: refinements in level 2
   * }
   * @endcode
   */
  void build_level_graphs();

  // Reuse the surface data already loaded by MeshPatches, with no CSV reads.
  // Partition ID ranges must match this tree.
  void build_level_graphs(const geometry::PatchMeshData& mesh);

  /**
   * Build from active leaf IDs and undirected leaf adjacency in global tree
   * IDs. Include isolated leaves in leaf_ids; empty tree buckets may be left
   * out. Invalid IDs, non-leaves and edges outside leaf_ids are rejected.
   * Duplicate edges and self edges are ignored. Replaces previous graphs
   * only on success.
   *
   * Level 0 contains the root. Cuts with no additional patches are omitted;
   * links connect the retained cuts directly. Root and terminal leaf cuts are
   * preserved (a single-leaf tree therefore has two cuts unless root is leaf).
   * Each retained level carries shallow leaves forward, partitioning the same
   * active leaves, even for unbalanced trees. Coarse patches are adjacent iff
   * an input edge connects their descendant leaves. The last level has no
   * child links. Empty input produces one empty graph.
   */
  void build_level_graphs(
      const std::vector<int>& leaf_ids,
      const std::vector<PatchEdge>& leaf_edges);

  // Zero until build_level_graphs() is called. Levels are indexed from zero.
  // Independent of legacy get_height(), which estimates subdivision height
  // from node types rather than counting these graph levels.
  int get_num_levels() const;

  // Original tree depth of a retained graph cut. Graph indices are compact;
  // semantic tree nodes/IDs and their descriptor transitions are unchanged.
  int get_level_depth(int level) const;

  // Throws std::out_of_range for an unavailable level. Rebuilding invalidates
  // references to graphs.
  const LevelGraph& get_level_graph(int level) const;

  // used in DFS
  std::vector<std::vector<int>> get_next_level_combination(
      const std::vector<int>& comb) const;

  // used in BFS
  std::vector<std::vector<int>> get_next_expand_combination(
      const std::vector<int>& comb, int expand_id) const;

  std::vector<Eigen::Vector3d> get_vertices_in_combination(
      const std::vector<int>& comb) const;

  bool is_valid(const std::vector<int>& comb) const;

  std::vector<int> get_child_ids(int node_id) const;

  bool is_checkable(const std::vector<int>& comb) const;

  int get_height() const;

  int get_l1() const;

  int get_l2() const;

  std::vector<int> get_leaves(int id) const;

  bool is_valid_combination(const std::vector<int>& comb) const;

private:
  std::string m_folder;
  // Global ID of the first node loaded from each partition, and its size.
  std::vector<std::pair<int, int>> m_partitions;
  PatchNode::Ptr m_root;
  std::vector<PatchNode::Ptr> m_nodes;
  std::vector<LevelGraph> m_level_graphs;
  std::vector<int> m_level_depths;
  std::vector<NodeData> m_node_data;
};
} // namespace crobot::patch_tree

#endif // PATCH_TREE_HPP
