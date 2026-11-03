#include "crobot/patch_tree/mesh_patch.hpp"

#include <algorithm>
#include <stdexcept>

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>

#include "crobot/geometry/mesh_utils.hpp"

namespace crobot::patch_tree {
using geometry::MeshUtils;

MeshPatch::MeshPatch(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
    const std::vector<Eigen::Vector3d>& icosahedron_triangle)
  : mesh_(mesh),
    icosahedron_triangle_(icosahedron_triangle),
    total_curvature_(0.0) {
}

void MeshPatch::add_triangle(
    int tid, const Eigen::Vector3d& normal, double solid_angle) {
  mesh_tids_.push_back(tid);
  mesh_triangle_normals_.push_back(normal);

  if (solid_angle < 0.0) {
    const auto& triangles      = mesh_->triangles_;
    const auto& vertex_normals = mesh_->vertex_normals_;

    int vid_1 = triangles[tid](0);
    int vid_2 = triangles[tid](1);
    int vid_3 = triangles[tid](2);

    Eigen::Vector3d n1 = vertex_normals[vid_1];
    Eigen::Vector3d n2 = vertex_normals[vid_2];
    Eigen::Vector3d n3 = vertex_normals[vid_3];

    solid_angle = MeshUtils::compute_solid_angle(n1, n2, n3);
  }

  solid_angles_.push_back(solid_angle);
  total_curvature_ += solid_angle;
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshPatch::get_surface_mesh()
    const {
  return mesh_tids_.empty() ? nullptr
                            : MeshUtils::extract_triangles(*mesh_, mesh_tids_);
}

std::pair<
    std::shared_ptr<open3d::geometry::TriangleMesh>,
    std::shared_ptr<open3d::geometry::TriangleMesh>>
MeshPatch::convert_to_mesh() const {

  if (mesh_tids_.empty()) {
    return {nullptr, nullptr};
  }

  auto patch_mesh = get_surface_mesh();

  // Create platonic solid mesh from vertex normals
  std::vector<Eigen::Vector3d> platonic_solid_vertices;
  for (int tid : mesh_tids_) {
    const auto& triangle       = mesh_->triangles_[tid];
    const auto& vertex_normals = mesh_->vertex_normals_;

    Eigen::Vector3d n1 = vertex_normals[triangle(0)].normalized();
    Eigen::Vector3d n2 = vertex_normals[triangle(1)].normalized();
    Eigen::Vector3d n3 = vertex_normals[triangle(2)].normalized();

    platonic_solid_vertices.push_back(n1);
    platonic_solid_vertices.push_back(n2);
    platonic_solid_vertices.push_back(n3);
  }

  // Remove duplicate vertices
  std::sort(
      platonic_solid_vertices.begin(),
      platonic_solid_vertices.end(),
      [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        if (a.x() != b.x())
          return a.x() < b.x();
        if (a.y() != b.y())
          return a.y() < b.y();
        return a.z() < b.z();
      });

  auto last = std::unique(
      platonic_solid_vertices.begin(),
      platonic_solid_vertices.end(),
      [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - b).norm() < 1e-6;
      });
  platonic_solid_vertices.erase(last, platonic_solid_vertices.end());

  // Create point cloud and mesh using ball pivoting
  auto point_cloud     = std::make_shared<open3d::geometry::PointCloud>();
  point_cloud->points_ = platonic_solid_vertices;
  point_cloud->EstimateNormals();

  std::vector<double> radii = {0.008, 0.015, 0.03, 0.08};
  auto platonic_solid_mesh
      = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(
          *point_cloud, radii);

  platonic_solid_mesh->Scale(14.0, Eigen::Vector3d::Zero());
  platonic_solid_mesh->ComputeVertexNormals();

  return {patch_mesh, platonic_solid_mesh};
}

std::vector<std::shared_ptr<MeshPatch>> MeshPatch::decompose() {
  const Eigen::Vector3d& v1 = icosahedron_triangle_[0];
  const Eigen::Vector3d& v2 = icosahedron_triangle_[1];
  const Eigen::Vector3d& v3 = icosahedron_triangle_[2];

  Eigen::Vector3d m1 = (v1 + v2) / 2.0;
  Eigen::Vector3d m2 = (v1 + v3) / 2.0;
  Eigen::Vector3d m3 = (v2 + v3) / 2.0;

  std::vector<std::shared_ptr<MeshPatch>> patches;
  patches.push_back(std::make_shared<MeshPatch>(
      mesh_, std::vector<Eigen::Vector3d>{v1, m1, m2}));
  patches.push_back(std::make_shared<MeshPatch>(
      mesh_, std::vector<Eigen::Vector3d>{m1, v2, m3}));
  patches.push_back(std::make_shared<MeshPatch>(
      mesh_, std::vector<Eigen::Vector3d>{m1, m2, m3}));
  patches.push_back(std::make_shared<MeshPatch>(
      mesh_, std::vector<Eigen::Vector3d>{m2, m3, v3}));

  for (size_t i = 0; i < mesh_tids_.size(); ++i) {
    int tid                   = mesh_tids_[i];
    const Eigen::Vector3d& nt = mesh_triangle_normals_[i];
    double solid_angle        = solid_angles_[i];

    bool added = false;
    for (auto& subpatch : patches) {
      if (!subpatch->is_intersected_with_vector(nt)) {
        continue;
      }
      subpatch->add_triangle(tid, nt, solid_angle);
      added = true;
      break;
    }

    if (!added) {
      throw std::runtime_error(
          "A ray intersecting with parent triangle does not intersect with any "
          "sub-triangles");
    }
  }

  return patches;
}

bool MeshPatch::is_total_curvature_exceeding_threshold() const {
  return total_curvature_ > MAX_TOTAL_CURVATURE_PER_PATCH;
}

bool MeshPatch::is_intersected_with_vector(
    const Eigen::Vector3d& vector) const {
  if (icosahedron_triangle_.size() < 3) {
    return false;
  }
  return MeshUtils::ray_intersects_triangle(
      vector,
      icosahedron_triangle_[0],
      icosahedron_triangle_[1],
      icosahedron_triangle_[2]);
}

} // namespace crobot::patch_tree
