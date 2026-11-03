#include "crobot/geometry/mesh_io.hpp"

#include <stdexcept>

#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/visualization/visualizer/Visualizer.h>
namespace crobot::geometry {
std::shared_ptr<open3d::geometry::TriangleMesh> MeshIO::load_mesh(
    const std::string& filename) {
  auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  if (!open3d::io::ReadTriangleMesh(filename, *mesh))
    throw std::runtime_error("Failed to load mesh from: " + filename);
  if (!mesh->HasVertexNormals())
    mesh->ComputeVertexNormals();
  return mesh;
}

void MeshIO::save_mesh(
    const std::string& filename, const open3d::geometry::TriangleMesh& mesh) {
  if (!open3d::io::WriteTriangleMesh(filename, mesh))
    throw std::runtime_error("Failed to export " + filename);
}

void MeshIO::visualize_mesh(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {
  open3d::visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow("Mesh Viewer", 1600, 900, 50, 50);
  visualizer.AddGeometry(mesh);
  visualizer.Run();
  visualizer.DestroyVisualizerWindow();
}

} // namespace crobot::geometry
