#pragma once
#include <memory>
#include <string>
namespace open3d::geometry {
class TriangleMesh;
}
namespace crobot::geometry {
class MeshIO {
public:
  static std::shared_ptr<open3d::geometry::TriangleMesh> load_mesh(
      const std::string& filename);
  static void save_mesh(
      const std::string& filename, const open3d::geometry::TriangleMesh& mesh);
  static void visualize_mesh(
      const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh);
};
} // namespace crobot::geometry
