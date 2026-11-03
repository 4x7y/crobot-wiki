#include "crobot/geometry/mesh_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace crobot::geometry {

std::vector<Eigen::Vector3d> MeshUtils::enclosing_normal_directions(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<int>& triangle_ids) {
  if (triangle_ids.empty()) {
    return {};
  }
  if (!mesh.HasVertexNormals()) {
    throw std::invalid_argument(
        "Normal bounds require inherited vertex normals");
  }
  Eigen::Vector3d lower
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper = -lower;
  for (int face : triangle_ids) {
    const auto& triangle = mesh.triangles_.at(face);
    for (int j = 0; j < 3; ++j) {
      const auto& normal = mesh.vertex_normals_.at(triangle[j]);
      if (!normal.allFinite() || normal.squaredNorm() == 0.0) {
        throw std::invalid_argument(
            "Normal bounds require finite nonzero normals");
      }
      lower = lower.cwiseMin(normal);
      upper = upper.cwiseMax(normal);
    }
  }
  std::vector<Eigen::Vector3d> corners;
  corners.reserve(8);
  for (int bits = 0; bits < 8; ++bits) {
    corners.emplace_back(
        (bits & 1) ? upper.x() : lower.x(),
        (bits & 2) ? upper.y() : lower.y(),
        (bits & 4) ? upper.z() : lower.z());
  }
  return corners;
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshUtils::extract_triangles(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<int>& triangle_ids,
    bool compact_vertices) {
  auto result = std::make_shared<open3d::geometry::TriangleMesh>();
  if (!compact_vertices) {
    result->vertices_ = mesh.vertices_;
    if (mesh.HasVertexNormals())
      result->vertex_normals_ = mesh.vertex_normals_;
    if (mesh.HasVertexColors())
      result->vertex_colors_ = mesh.vertex_colors_;
  }
  std::map<int, int> remap;
  result->triangles_.reserve(triangle_ids.size());
  for (int id : triangle_ids) {
    auto triangle = mesh.triangles_.at(id);
    if (compact_vertices) {
      for (int j = 0; j < 3; ++j) {
        const int source = triangle[j];
        auto inserted    = remap.emplace(source, remap.size());
        if (inserted.second) {
          result->vertices_.push_back(mesh.vertices_.at(source));
          if (mesh.HasVertexNormals())
            result->vertex_normals_.push_back(mesh.vertex_normals_[source]);
          if (mesh.HasVertexColors())
            result->vertex_colors_.push_back(mesh.vertex_colors_[source]);
        }
        triangle[j] = inserted.first->second;
      }
    }
    result->triangles_.push_back(triangle);
  }
  if (!result->triangles_.empty()) {
    if (!result->HasVertexNormals())
      result->ComputeVertexNormals();
    else
      result->ComputeTriangleNormals();
  }
  return result;
}

std::vector<int> MeshUtils::expand_vertex_ids_by_radius(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
    const std::vector<int>& seed_vertex_ids,
    double radius) {

  std::unordered_set<int> expanded_ids;
  const auto& vertices = mesh->vertices_;

  // Build KD-tree for efficient neighbor search
  for (int seed_id : seed_vertex_ids) {
    const auto& seed_vertex = vertices[seed_id];

    for (size_t i = 0; i < vertices.size(); ++i) {
      double dist = (vertices[i] - seed_vertex).norm();
      if (dist <= radius) {
        expanded_ids.insert(static_cast<int>(i));
      }
    }
  }

  std::vector<int> result(expanded_ids.begin(), expanded_ids.end());
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<int> MeshUtils::filter_dense_red_vertices(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
    const std::vector<int>& red_vertex_ids,
    double radius,
    int k) {

  std::vector<int> filtered_ids;
  const auto& vertices = mesh->vertices_;

  for (int red_id : red_vertex_ids) {
    const auto& red_vertex = vertices[red_id];
    int neighbor_count     = 0;

    for (int other_red_id : red_vertex_ids) {
      double dist = (vertices[other_red_id] - red_vertex).norm();
      if (dist <= radius) {
        neighbor_count++;
      }
    }

    if (neighbor_count >= k) {
      filtered_ids.push_back(red_id);
    }
  }

  return filtered_ids;
}

double MeshUtils::compute_solid_angle(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& p3) {

  double a = p1.dot(p2.cross(p3));
  double b = p1.norm() * p2.norm() * p3.norm() + p1.dot(p2) * p3.norm()
             + p1.dot(p3) * p2.norm() + p2.dot(p3) * p1.norm();

  if (!p1.allFinite() || !p2.allFinite() || !p3.allFinite() || p1.norm() == 0
      || p2.norm() == 0 || p3.norm() == 0)
    throw std::invalid_argument("Solid angle requires finite nonzero vectors");
  return 2.0 * std::atan2(std::abs(a), b);
}

bool MeshUtils::ray_intersects_triangle(
    const Eigen::Vector3d& dir,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2) {

  const double EPSILON = 1e-10;

  Eigen::Vector3d u = v1 - v0;
  Eigen::Vector3d v = v2 - v0;
  Eigen::Vector3d n = u.cross(v);
  if (!dir.allFinite() || dir.norm() == 0 || n.squaredNorm() == 0) {
    return false;
  }
  Eigen::Vector3d normalized_dir = dir.normalized();

  Eigen::Vector3d w0 = -v0;
  double a           = -n.dot(w0);
  double b           = n.dot(normalized_dir);

  if (std::abs(b) <= 1e-14 * n.norm()) {
    return false;
  }

  double pos = a / b;
  if (pos < 0)
    return false;
  Eigen::Vector3d point = pos * normalized_dir;

  double uu = u.dot(u);
  double uv = u.dot(v);
  double vv = v.dot(v);

  Eigen::Vector3d w = point - v0;
  double wu         = w.dot(u);
  double wv         = w.dot(v);

  double dc = uv * uv - uu * vv;
  if (dc == 0)
    return false;

  double s = (uv * wv - vv * wu) / dc;
  if (s < -EPSILON || s > 1.0 + EPSILON) {
    return false;
  }

  double t = (uv * wu - uu * wv) / dc;
  if (t < -EPSILON || (s + t) > 1.0 + EPSILON) {
    return false;
  }

  return true;
}

std::shared_ptr<open3d::geometry::TriangleMesh>
MeshUtils::generate_subdivided_mesh(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
    int number_of_iterations) {

  auto subdivided_mesh
      = std::make_shared<open3d::geometry::TriangleMesh>(*mesh);
  subdivided_mesh->ComputeVertexNormals();

  for (int i = 0; i < number_of_iterations; ++i) {
    *subdivided_mesh = *subdivided_mesh->SubdivideLoop(1);
  }

  return subdivided_mesh;
}

std::pair<Eigen::Matrix3d, Eigen::Matrix3d>
MeshUtils::calculate_zy_rotation_for_arrow(const Eigen::Vector3d& vec) {

  double gamma = std::atan2(vec.y(), vec.x());
  Eigen::Matrix3d Rz;
  Rz << std::cos(gamma), -std::sin(gamma), 0, std::sin(gamma), std::cos(gamma),
      0, 0, 0, 1;

  Eigen::Vector3d vec_rotated = Rz.transpose() * vec;
  double beta                 = std::atan2(vec_rotated.x(), vec_rotated.z());

  Eigen::Matrix3d Ry;
  Ry << std::cos(beta), 0, std::sin(beta), 0, 1, 0, -std::sin(beta), 0,
      std::cos(beta);

  return {Rz, Ry};
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshUtils::create_arrow(
    const Eigen::Vector3d& end, const Eigen::Vector3d& origin, double scale) {

  Eigen::Vector3d vec = end - origin;
  double size         = vec.norm();

  auto [Rz, Ry] = calculate_zy_rotation_for_arrow(vec);

  auto mesh = open3d::geometry::TriangleMesh::CreateArrow(
      size / 30.0 * scale,        // cylinder_radius
      size * (1.0 - 0.2 * scale), // cylinder_height
      size / 17.5 * scale,        // cone_radius
      size * 0.2 * scale          // cone_height
  );

  mesh->Rotate(Ry, Eigen::Vector3d::Zero());
  mesh->Rotate(Rz, Eigen::Vector3d::Zero());
  mesh->Translate(origin);

  // Set red color
  mesh->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));

  return mesh;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
MeshUtils::get_vertices_and_normals(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {
  if (!mesh)
    throw std::invalid_argument("Null submesh");
  std::set<int> ids;
  for (const auto& t : mesh->triangles_)
    for (int j = 0; j < 3; ++j)
      ids.insert(t[j]);
  std::vector<Eigen::Vector3d> vertices, normals;
  for (int id : ids) {
    vertices.push_back(mesh->vertices_.at(id));
    normals.push_back(mesh->vertex_normals_.at(id));
  }
  return {vertices, normals};
}

} // namespace crobot::geometry
