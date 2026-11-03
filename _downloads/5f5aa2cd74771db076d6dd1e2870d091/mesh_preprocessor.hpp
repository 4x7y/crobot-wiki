#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

namespace open3d {
namespace geometry {
class TriangleMesh;
}
} // namespace open3d

namespace crobot::geometry {

struct MeshPreparationOptions
{
  bool weld_exact_vertices = true;
  bool orient_triangles    = true;
  // Treat nested closed shells as boundaries of one solid, including cavities.
  // Disable for independent solids that happen to be nested geometrically.
  bool orient_nested_shells             = true;
  int normal_smoothing_iterations       = 2;
  double normal_smoothing_angle_degrees = 30.0;
  // Square units of the input mesh. Zero removes only zero-area faces.
  double min_triangle_area = 0.0;
};

enum class OriginalFaceDisposition
{
  Retained,
  Duplicate,
  Degenerate
};

struct MeshPreparationReport
{
  std::size_t original_vertices           = 0;
  std::size_t original_faces              = 0;
  std::size_t prepared_vertices           = 0;
  std::size_t prepared_faces              = 0;
  std::size_t welded_vertices             = 0;
  std::size_t degenerate_faces_removed    = 0;
  std::size_t duplicate_faces_removed     = 0;
  std::size_t flipped_faces               = 0;
  std::size_t connected_components        = 0;
  std::size_t orientation_components      = 0;
  std::size_t outward_oriented_components = 0;
  std::size_t inward_oriented_components  = 0;
  std::size_t nested_closed_components    = 0;
  std::size_t intersecting_shell_pairs    = 0;
  std::size_t ambiguous_closed_components = 0;
  std::size_t boundary_edges              = 0;
  std::size_t nonmanifold_edges           = 0;
  std::size_t nonmanifold_vertices        = 0;
  std::size_t orientation_conflicts       = 0;
  std::size_t inconsistent_manifold_edges = 0;
  std::size_t unreferenced_vertices       = 0;
  std::size_t fallback_normal_vertices    = 0;
  std::size_t feature_vertices            = 0;
  std::size_t smoothed_vertices           = 0;
  // Topology diagnostics above describe cleanup; prepared counts are current.
  std::size_t refinement_rounds         = 0;
  std::size_t refinement_added_vertices = 0;
  std::size_t refinement_added_faces    = 0;
  std::vector<std::array<int, 2>> nonmanifold_edge_vertex_ids;
  std::vector<std::array<int, 2>> orientation_conflict_edge_vertex_ids;
  std::vector<int> nonmanifold_vertex_ids;
  std::vector<int> ambiguous_closed_component_ids;
};

struct PreparedMesh
{
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
  MeshPreparationReport report;
  std::vector<int> original_vertex_to_prepared;
  // Includes the retained source face and every coincident duplicate.
  std::vector<std::vector<int>> face_source_ids;
  // First representative after refinement. Degenerate source faces map to -1.
  std::vector<int> original_face_to_prepared;
  // Full correspondence, including duplicates and one-to-many refinement.
  std::vector<std::vector<int>> original_face_to_prepared_faces;
  std::vector<OriginalFaceDisposition> original_face_disposition;
};

/**
 * Prepare geometry without moving vertices or discarding small components.
 *
 * Rejects nonfinite vertices, invalid face indices, and invalid options.
 * Welding is exact; near-coincident surfaces are not merged. Orientation is
 * propagated only across manifold edges. Nonmanifold topology and orientation
 * conflicts are reported and retained. Disjoint closed shells are oriented by
 * containment parity: outer shells point outward and cavity shells inward.
 * This assumes one solid with cavities; orient_nested_shells=false instead
 * treats nested shells as independent solids. Intersecting/touching shells or
 * uncertain containment retain their consistent input side and are reported.
 * Signed-volume/containment checks do not prove absence of self intersections.
 *
 * Normals are recomputed from the prepared faces and optionally filtered with
 * an angular gate. Boundary, nonmanifold, and sharp-edge vertices are held
 * fixed during filtering. A single vertex normal cannot express multiple
 * normals at a crease; the crease's geometry and its unfiltered normal are
 * preserved. Unreferenced vertices receive a reported unit-Z fallback normal.
 */
PreparedMesh prepare_mesh(
    const open3d::geometry::TriangleMesh& input,
    const MeshPreparationOptions& options = {});

} // namespace crobot::geometry
