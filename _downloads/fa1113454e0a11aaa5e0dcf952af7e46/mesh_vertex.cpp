
#include "crobot/geometry/mesh_vertex.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace crobot::geometry;

//==============================================================================
MeshVertex::MeshVertex(
    int id, const math::Vector3d& pos, const math::Vector3d& normal, int label)
  : m_id(id), m_pos(pos), m_normal(normal), m_label(label) {
  m_neighbors.clear();

  // initialize level
  if (label <= 0) {
    m_level = 0;
  }
  else {
    m_level = -1;
  }
}

//==============================================================================
void MeshVertex::addNeighbor(MeshVertex::Ptr neighbor) {
  // check if neighbor already exists
  for (size_t i = 0; i < m_neighbors.size(); i++) {
    if (m_neighbors[i] == neighbor) {
      return;
    }
  }
  m_neighbors.push_back(neighbor);
}
//==============================================================================
std::vector<MeshVertex::Ptr> MeshVertex::get_neighbors() const {
  return m_neighbors;
}

//==============================================================================
std::vector<int> MeshVertex::get_neighbor_ids() const {
  std::vector<int> neighbor_ids;
  for (size_t i = 0; i < m_neighbors.size(); i++) {
    neighbor_ids.push_back(m_neighbors[i]->get_id());
  }
  return neighbor_ids;
}

//==============================================================================
std::vector<int> MeshVertex::get_neighbor_ids_within_same_patch() const {
  std::vector<int> neighbor_ids;
  for (size_t i = 0; i < m_neighbors.size(); i++) {
    if (m_neighbors[i]->get_label() == m_label)
      neighbor_ids.push_back(m_neighbors[i]->get_id());
  }
  return neighbor_ids;
}

//==============================================================================
std::vector<MeshVertex::Ptr> MeshVertex::get_unleveled_neighbors() const {
  std::vector<MeshVertex::Ptr> unleveled_neighbors;
  for (size_t i = 0; i < m_neighbors.size(); i++) {
    if (!m_neighbors[i]->is_level_assigned()) {
      unleveled_neighbors.push_back(m_neighbors[i]);
    }
  }
  return unleveled_neighbors;
}

//==============================================================================
int MeshVertex::get_id() const {
  return m_id;
}

//==============================================================================

math::Vector3d MeshVertex::get_pos() const {
  return m_pos;
}
//==============================================================================
math::Vector3d MeshVertex::get_normal() const {
  return m_normal;
}
//==============================================================================
math::Vector3d MeshVertex::get_normal_inward() const {
  return -m_normal;
}
//==============================================================================
int MeshVertex::get_label() const {
  return m_label;
}
//==============================================================================
int MeshVertex::get_level() const {
  return m_level;
}
//==============================================================================
void MeshVertex::set_level(int level) {
  CROBOT_ASSERT(!is_level_assigned(), "Level already assigned");
  m_level = level;
}
//==============================================================================
bool MeshVertex::is_level_assigned() const {
  return m_level >= 0;
}
