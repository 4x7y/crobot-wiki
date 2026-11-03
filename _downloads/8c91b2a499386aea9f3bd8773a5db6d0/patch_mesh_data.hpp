#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace crobot::geometry {

/**
 * Shared, unscaled surface data for saved patch partitions. PatchTree uses
 * face adjacency; MeshPatches adds contact vertices and patch centers. This
 * class owns CSV parsing and local-to-global IDs for both consumers.
 * MeshPatch, in patch_tree/, is instead a region being actively subdivided.
 */
class PatchMeshData {
public:
  using ConstPtr = std::shared_ptr<const PatchMeshData>;
  using Edge     = std::pair<int, int>;

  struct Partition
  {
    int first_patch_id;
    int patch_count;
    int first_vertex_id;
    int vertex_count;
  };

  struct PatchAdjacency
  {
    std::vector<int> patch_ids;
    std::vector<Edge> edges;
  };

  struct VertexAttributes
  {
    std::vector<Eigen::Vector3d> normals;
    std::vector<int> patch_ids;
  };

  // Both overloads read vertices.csv, faces.csv and face_labels.csv from each
  // partition. This overload reads node counts from patch_offsets.csv.
  // Vertex offsets and vertex edges
  // are derived from the mesh, so vertex_offsets.csv/adj_list.csv are
  // redundant.
  static ConstPtr load(const std::string& folder, int num_partitions);

  // A tree that already knows its partition sizes need not read offset files.
  static ConstPtr load(
      const std::string& folder, const std::vector<int>& partition_sizes);

  const std::string& get_folder() const;
  const std::vector<Partition>& get_partitions() const;
  const std::vector<Eigen::Vector3d>& get_vertices() const;

  // Includes root/internal/empty IDs, matching the legacy MeshPatches API.
  int get_num_patches() const;

  // Vertex IDs stay distinct across partitions for contact lookup.
  std::vector<Edge> get_vertex_edges() const;

  // Patch adjacency welds exact endpoint coordinates across partitions.
  // Unassigned faces (-1) do not create edges; point contacts do not connect.
  PatchAdjacency get_patch_adjacency() const;

  // Contact-only files are optional until this method is called. Normals are
  // normalized; positive vertex labels use the same global IDs as face labels.
  VertexAttributes load_vertex_attributes() const;

private:
  PatchMeshData() = default;

  std::string m_folder;
  std::vector<Partition> m_partitions;
  std::vector<Eigen::Vector3d> m_vertices;
  std::vector<std::array<int, 3>> m_faces;
  std::vector<int> m_face_labels;
  int m_num_patches = 1;
};

} // namespace crobot::geometry
