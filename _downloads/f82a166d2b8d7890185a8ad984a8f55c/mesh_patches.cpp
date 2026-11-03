#include "crobot/geometry/mesh_patches.hpp"

#include <stdexcept>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace crobot::geometry;

//==============================================================================
MeshPatches::MeshPatches(const std::string& folder, int num, double scale)
  : MeshPatches(PatchMeshData::load(folder, num), scale) {
}

//==============================================================================
MeshPatches::MeshPatches(PatchMeshData::ConstPtr mesh_data, double scale)
  : m_mesh_data(std::move(mesh_data)), m_scale(scale) {
  if (!m_mesh_data) {
    throw std::invalid_argument("MeshPatches requires surface data");
  }
  const auto& vertices  = m_mesh_data->get_vertices();
  const auto attributes = m_mesh_data->load_vertex_attributes();
  m_patch_centers.resize(m_mesh_data->get_num_patches(), nullptr);
  std::vector<std::vector<MeshVertex::Ptr>> boundaries(m_patch_centers.size());

  for (size_t i = 0; i < vertices.size(); ++i) {
    m_vertices.push_back(std::make_shared<MeshVertex>(
        i,
        vertices[i] * m_scale,
        attributes.normals[i],
        attributes.patch_ids[i]));
  }

  for (const auto& edge : m_mesh_data->get_vertex_edges()) {
    auto first  = m_vertices[edge.first];
    auto second = m_vertices[edge.second];
    first->addNeighbor(second);
    second->addNeighbor(first);
    auto seed_boundary = [&](const MeshVertex::Ptr& boundary,
                             const MeshVertex::Ptr& interior) {
      if (boundary->get_level() == 0 && !interior->is_level_assigned()
          && interior->get_label() > 0) {
        interior->set_level(1);
        boundaries[interior->get_label()].push_back(interior);
      }
    };
    seed_boundary(first, second);
    seed_boundary(second, first);
  }

  for (size_t i = 0; i < boundaries.size(); ++i) {
    if (!boundaries[i].empty()) {
      m_patch_centers[i] = find_patch_center(boundaries[i]);
    }
  }
}

//==============================================================================
PatchMeshData::ConstPtr MeshPatches::get_mesh_data() const {
  return m_mesh_data;
}

//==============================================================================
std::string MeshPatches::get_folder() const {
  return m_mesh_data->get_folder();
}
//==============================================================================
MeshVertex::ConstPtr MeshPatches::get_patch_center(int patch_label) const {
  return m_patch_centers.at(patch_label);
}

//==============================================================================
bool MeshPatches::is_center_defined(std::vector<int> patch_labels) const {
  for (int label : patch_labels) {

    if (label <= 0)
      return false;

    if (get_patch_center(label) == nullptr)
      return false;
  }
  return true;
}

//==============================================================================
int MeshPatches::get_num_patches() const {
  return m_mesh_data->get_num_patches();
}

//==============================================================================
MeshVertex::ConstPtr MeshPatches::get_vertex(int index) const {
  return m_vertices.at(index);
}

//==============================================================================
MeshVertex::ConstPtr MeshPatches::find_patch_center(
    const std::vector<MeshVertex::Ptr>& patch_level_boundary) {

  // find most interior vertex of the patch
  std::vector<MeshVertex::Ptr> next_level_boundary;
  int next_level = patch_level_boundary[0]->get_level() + 1;

  if (patch_level_boundary.size() == 1) {
    return patch_level_boundary[0];
  }

  for (size_t i = 0; i < patch_level_boundary.size(); i++) {
    auto neighbors = patch_level_boundary[i]->get_unleveled_neighbors();
    for (size_t j = 0; j < neighbors.size(); j++) {
      neighbors[j]->set_level(next_level);
      next_level_boundary.push_back(neighbors[j]);
    }
  }

  if (next_level_boundary.size() == 0) {

    // find the most center point in patch_level_boundary
    double min_dist = 1e10;

    math::Vector3d center = math::Vector3d::Zero();
    for (size_t i = 0; i < patch_level_boundary.size(); i++) {
      center += patch_level_boundary[i]->get_pos();
    }
    center /= patch_level_boundary.size();

    MeshVertex::Ptr center_point = nullptr;
    for (size_t i = 0; i < patch_level_boundary.size(); i++) {
      double dist = (patch_level_boundary[i]->get_pos() - center).norm();
      if (dist < min_dist) {
        min_dist     = dist;
        center_point = patch_level_boundary[i];
      }
    }

    return center_point;
  }

  return find_patch_center(next_level_boundary);
}

//==============================================================================
