#ifndef MESH_VERTEX_HPP
#define MESH_VERTEX_HPP

#include <memory>
#include <vector>

#include "crobot/math/matrix.hpp"

namespace crobot::geometry {

class MeshVertex {
public:
  using Ptr      = std::shared_ptr<MeshVertex>;
  using ConstPtr = std::shared_ptr<const MeshVertex>;

  MeshVertex(
      int id,
      const math::Vector3d& pos,
      const math::Vector3d& normal,
      int label);

  void addNeighbor(MeshVertex::Ptr neighbor);

  std::vector<MeshVertex::Ptr> get_neighbors() const;

  std::vector<int> get_neighbor_ids() const;

  std::vector<int> get_neighbor_ids_within_same_patch() const;

  std::vector<MeshVertex::Ptr> get_unleveled_neighbors() const;

  int get_id() const;
  math::Vector3d get_pos() const;
  math::Vector3d get_normal() const;
  math::Vector3d get_normal_inward() const;
  int get_label() const;
  int get_level() const;
  void set_level(int level);
  bool is_level_assigned() const;

private:
  int m_id;
  math::Vector3d m_pos;
  math::Vector3d m_normal;

  // zero: boundary vertex
  // 1 - k: patch label
  int m_label;

  // the most interior vertex has highest level
  // 1 if next to boundary
  // -1 if not assigned
  int m_level;

  std::vector<MeshVertex::Ptr> m_neighbors;
};
} // namespace crobot::geometry

#endif // MESH_VERTEX_HPP
