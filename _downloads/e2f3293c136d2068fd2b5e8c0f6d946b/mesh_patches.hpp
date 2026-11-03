#ifndef MESH_PATCHES_HPP
#define MESH_PATCHES_HPP

#include <memory>
#include <string>
#include <vector>

#include "crobot/geometry/mesh_vertex.hpp"
#include "crobot/geometry/patch_mesh_data.hpp"

namespace crobot::geometry {

// Shared view of saved patch geometry: vertex neighbors, boundary distances,
// and representative centers. Contains no hand or grasp-search policy.
class MeshPatches {
public:
  using Ptr      = std::shared_ptr<MeshPatches>;
  using ConstPtr = std::shared_ptr<const MeshPatches>;

  // Requires saved surface faces/labels as well as normals and vertex labels.
  MeshPatches(const std::string& folder, int num, double scale);

  MeshPatches(PatchMeshData::ConstPtr mesh_data, double scale);

  // Share this surface with PatchTree::build_level_graphs(*get_mesh_data()).
  PatchMeshData::ConstPtr get_mesh_data() const;

  MeshVertex::ConstPtr get_patch_center(int patch_label) const;

  bool is_center_defined(std::vector<int> patch_labels) const;

  double get_scale() const {
    return m_scale;
  }

  int get_num_patches() const;

  MeshVertex::ConstPtr get_vertex(int index) const;

  std::string get_folder() const;

private:
  MeshVertex::ConstPtr find_patch_center(
      const std::vector<MeshVertex::Ptr>& patch_level_boundary);

  PatchMeshData::ConstPtr m_mesh_data;
  std::vector<MeshVertex::Ptr> m_vertices;

  double m_scale;

  // most interior vertex of each patch
  std::vector<MeshVertex::ConstPtr> m_patch_centers;
};
} // namespace crobot::geometry

#endif // MESH_PATCHES_HPP
