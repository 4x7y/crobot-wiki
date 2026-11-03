#include "crobot/geometry/mesh_refinement.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

#include <Eigen/Core>
#include <open3d/geometry/TriangleMesh.h>

namespace crobot::geometry {
namespace {

using Edge = std::array<int, 2>;

Edge edge_key(int a, int b) {
  return {std::min(a, b), std::max(a, b)};
}

} // namespace

std::vector<int> refine_marked_faces(
    PreparedMesh& prepared, const std::vector<int>& marked_faces) {
  if (!prepared.mesh)
    throw std::invalid_argument("Cannot refine a null prepared mesh");
  const auto& old = *prepared.mesh;
  if (old.triangles_.size() != prepared.face_source_ids.size()
      || prepared.original_face_to_prepared.size()
             != prepared.original_face_disposition.size())
    throw std::invalid_argument("Invalid prepared-face correspondence");
  std::map<Edge, int> midpoints;
  for (int face : marked_faces) {
    if (face < 0 || static_cast<std::size_t>(face) >= old.triangles_.size())
      throw std::out_of_range("Invalid face marked for refinement");
    const auto& triangle = old.triangles_[face];
    for (int k = 0; k < 3; ++k)
      midpoints.emplace(edge_key(triangle[k], triangle[(k + 1) % 3]), -1);
  }
  if (midpoints.empty()) {
    std::vector<int> identity(old.triangles_.size());
    std::iota(identity.begin(), identity.end(), 0);
    return identity;
  }
  if (!old.HasVertexNormals())
    throw std::invalid_argument(
        "Refinement requires the prepared vertex-normal field");
  if (old.vertices_.size() + midpoints.size()
          > static_cast<std::size_t>(std::numeric_limits<int>::max())
      || old.triangles_.size()
             > static_cast<std::size_t>(std::numeric_limits<int>::max() / 4))
    throw std::length_error("Refinement exceeds supported integer index range");

  auto refined      = std::make_shared<open3d::geometry::TriangleMesh>(old);
  const bool colors = old.HasVertexColors();
  const bool uvs    = old.HasTriangleUvs();
  const bool materials
      = old.triangle_material_ids_.size() == old.triangles_.size();
  for (auto& entry : midpoints) {
    const int a = entry.first[0], b = entry.first[1];
    const Eigen::Vector3d point
        = old.vertices_[a] * 0.5 + old.vertices_[b] * 0.5;
    const Eigen::Vector3d normal_sum
        = old.vertex_normals_[a] + old.vertex_normals_[b];
    const double normal_length = normal_sum.stableNorm();
    if (!point.allFinite() || point == old.vertices_[a]
        || point == old.vertices_[b])
      throw std::runtime_error(
          "Refinement exhausted edge coordinate precision");
    if (!normal_sum.allFinite() || normal_length <= 1e-12)
      throw std::domain_error(
          "Refinement encountered antipodal or invalid endpoint normals");
    entry.second = static_cast<int>(refined->vertices_.size());
    refined->vertices_.push_back(point);
    refined->vertex_normals_.push_back(normal_sum / normal_length);
    if (colors)
      refined->vertex_colors_.push_back(
          (old.vertex_colors_[a] + old.vertex_colors_[b]) * 0.5);
  }
  refined->triangles_.clear();
  refined->triangle_normals_.clear();
  refined->triangle_uvs_.clear();
  refined->triangle_material_ids_.clear();
  refined->adjacency_list_.clear();
  std::vector<int> new_to_old;
  std::vector<std::vector<int>> source_ids;
  for (std::size_t face = 0; face < old.triangles_.size(); ++face) {
    const auto& triangle = old.triangles_[face];
    // Local IDs 0..2 are old corners; 3..5 are edge midpoints (01,12,20).
    std::array<int, 6> vertices{
        triangle[0], triangle[1], triangle[2], -1, -1, -1};
    std::array<Eigen::Vector2d, 6> texture;
    int split_edges = 0;
    for (int k = 0; k < 3; ++k) {
      const auto found
          = midpoints.find(edge_key(triangle[k], triangle[(k + 1) % 3]));
      if (found != midpoints.end()) {
        vertices[3 + k] = found->second;
        ++split_edges;
      }
      if (uvs) {
        texture[k]     = old.triangle_uvs_[3 * face + k];
        texture[3 + k] = old.triangle_uvs_[3 * face + k] * 0.5
                         + old.triangle_uvs_[3 * face + (k + 1) % 3] * 0.5;
      }
    }
    const auto emit = [&](int a, int b, int c) {
      const Eigen::Vector3i child(vertices[a], vertices[b], vertices[c]);
      const Eigen::Vector3d cross
          = (refined->vertices_[child[1]] - refined->vertices_[child[0]])
                .cross(
                    refined->vertices_[child[2]]
                    - refined->vertices_[child[0]]);
      const double length = cross.stableNorm();
      if (!cross.allFinite() || length == 0.0)
        throw std::runtime_error(
            "Refinement produced a numerically degenerate face");
      refined->triangles_.push_back(child);
      refined->triangle_normals_.push_back(cross / length);
      new_to_old.push_back(static_cast<int>(face));
      source_ids.push_back(prepared.face_source_ids[face]);
      if (uvs) {
        refined->triangle_uvs_.push_back(texture[a]);
        refined->triangle_uvs_.push_back(texture[b]);
        refined->triangle_uvs_.push_back(texture[c]);
      }
      if (materials)
        refined->triangle_material_ids_.push_back(
            old.triangle_material_ids_[face]);
    };
    if (split_edges == 0)
      emit(0, 1, 2);
    else if (split_edges == 1) {
      int a = 0;
      while (vertices[3 + a] < 0)
        ++a;
      const int b = (a + 1) % 3, c = (a + 2) % 3, ab = 3 + a;
      emit(a, ab, c);
      emit(ab, b, c);
    }
    else if (split_edges == 2) {
      int a = 0;
      while (vertices[3 + a] < 0 || vertices[3 + (a + 2) % 3] < 0)
        ++a;
      const int b = (a + 1) % 3, c = (a + 2) % 3;
      const int ab = 3 + a, ca = 3 + (a + 2) % 3;
      emit(a, ab, ca);
      const double first_diagonal
          = (refined->vertices_[vertices[ab]] - refined->vertices_[vertices[c]])
                .stableNorm();
      const double second_diagonal
          = (refined->vertices_[vertices[b]] - refined->vertices_[vertices[ca]])
                .stableNorm();
      if (first_diagonal < second_diagonal
          || (first_diagonal == second_diagonal
              && edge_key(vertices[ab], vertices[c])
                     < edge_key(vertices[b], vertices[ca]))) {
        emit(ab, b, c);
        emit(ab, c, ca);
      }
      else {
        emit(ab, b, ca);
        emit(b, c, ca);
      }
    }
    else {
      emit(0, 3, 5);
      emit(3, 1, 4);
      emit(5, 4, 2);
      emit(3, 4, 5);
    }
  }
  std::vector<std::vector<int>> originals(
      prepared.original_face_to_prepared.size());
  for (std::size_t face = 0; face < source_ids.size(); ++face)
    for (int original : source_ids[face]) {
      if (original < 0
          || static_cast<std::size_t>(original) >= originals.size())
        throw std::invalid_argument(
            "Invalid original face in prepared-face correspondence");
      originals[original].push_back(static_cast<int>(face));
    }
  std::vector<int> representatives(originals.size(), -1);
  for (std::size_t original = 0; original < originals.size(); ++original)
    if (!originals[original].empty())
      representatives[original] = originals[original].front();
  prepared.report.refinement_added_vertices
      += refined->vertices_.size() - old.vertices_.size();
  prepared.report.refinement_added_faces
      += refined->triangles_.size() - old.triangles_.size();
  ++prepared.report.refinement_rounds;
  prepared.report.prepared_vertices        = refined->vertices_.size();
  prepared.report.prepared_faces           = refined->triangles_.size();
  prepared.mesh                            = std::move(refined);
  prepared.face_source_ids                 = std::move(source_ids);
  prepared.original_face_to_prepared       = std::move(representatives);
  prepared.original_face_to_prepared_faces = std::move(originals);
  return new_to_old;
}

} // namespace crobot::geometry
