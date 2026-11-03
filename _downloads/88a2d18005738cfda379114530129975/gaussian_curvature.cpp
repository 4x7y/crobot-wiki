#include "crobot/geometry/gaussian_curvature.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <unordered_set>

#include <open3d/geometry/TriangleMesh.h>

namespace crobot::geometry {

double GaussianCurvature::compute_angle(
    const Eigen::Vector3d& u, const Eigen::Vector3d& v) {
  double uu        = u.squaredNorm();
  double vv        = v.squaredNorm();
  double uv        = u.dot(v);
  double cos_angle = std::clamp(uv / (std::sqrt(uu * vv) + 1e-15), -1.0, 1.0);
  return std::acos(cos_angle);
}

std::unordered_set<int> GaussianCurvature::detect_boundary_vertices(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {

  const auto& triangles = mesh->triangles_;

  // Count edge occurrences
  std::map<std::pair<int, int>, int> edge_count;

  for (const auto& tri : triangles) {
    std::vector<std::pair<int, int>> edges
        = {{std::min(tri(0), tri(1)), std::max(tri(0), tri(1))},
           {std::min(tri(1), tri(2)), std::max(tri(1), tri(2))},
           {std::min(tri(2), tri(0)), std::max(tri(2), tri(0))}};

    for (const auto& edge : edges) {
      edge_count[edge]++;
    }
  }

  // Boundary edges appear only once
  std::unordered_set<int> boundary_vertices;
  for (const auto& [edge, count] : edge_count) {
    if (count == 1) {
      boundary_vertices.insert(edge.first);
      boundary_vertices.insert(edge.second);
    }
  }

  return boundary_vertices;
}

Eigen::VectorXd GaussianCurvature::compute_angle_deficit(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {

  const auto& vertices  = mesh->vertices_;
  const auto& triangles = mesh->triangles_;
  int n                 = static_cast<int>(vertices.size());

  Eigen::VectorXd angle_sum   = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd vertex_area = Eigen::VectorXd::Zero(n);

  // Compute angles and areas
  for (const auto& tri : triangles) {
    int v0_idx = tri(0);
    int v1_idx = tri(1);
    int v2_idx = tri(2);

    Eigen::Vector3d v0 = vertices[v0_idx];
    Eigen::Vector3d v1 = vertices[v1_idx];
    Eigen::Vector3d v2 = vertices[v2_idx];

    Eigen::Vector3d e0 = v1 - v0;
    Eigen::Vector3d e1 = v2 - v1;
    Eigen::Vector3d e2 = v0 - v2;

    // Angles at each vertex
    double A0 = compute_angle(-e2, e0); // at v0
    double A1 = compute_angle(-e0, e1); // at v1
    double A2 = compute_angle(-e1, e2); // at v2

    angle_sum(v0_idx) += A0;
    angle_sum(v1_idx) += A1;
    angle_sum(v2_idx) += A2;

    // Triangle area using cross product
    double tri_area     = 0.5 * (v1 - v0).cross(v2 - v0).norm();
    double area_contrib = tri_area / 3.0;

    vertex_area(v0_idx) += area_contrib;
    vertex_area(v1_idx) += area_contrib;
    vertex_area(v2_idx) += area_contrib;
  }

  // Detect boundary vertices
  auto boundary_verts = detect_boundary_vertices(mesh);

  // Compute Gaussian curvature using angle deficit
  Eigen::VectorXd curvature(n);
  for (int i = 0; i < n; ++i) {
    double target = boundary_verts.count(i) > 0 ? M_PI : 2.0 * M_PI;
    curvature(i)  = (target - angle_sum(i)) / (vertex_area(i) + 1e-15);
  }

  return curvature;
}

std::vector<std::unordered_set<int>> GaussianCurvature::build_vertex_adjacency(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh) {

  int n = static_cast<int>(mesh->vertices_.size());
  std::vector<std::unordered_set<int>> adjacency(n);

  for (const auto& tri : mesh->triangles_) {
    int a = tri(0);
    int b = tri(1);
    int c = tri(2);

    adjacency[a].insert(b);
    adjacency[a].insert(c);
    adjacency[b].insert(a);
    adjacency[b].insert(c);
    adjacency[c].insert(a);
    adjacency[c].insert(b);
  }

  return adjacency;
}

std::unordered_set<int> GaussianCurvature::get_kring_neighbors(
    const std::vector<std::unordered_set<int>>& adjacency,
    int start_vertex,
    int k) {

  std::unordered_set<int> seen;
  std::unordered_set<int> frontier;

  seen.insert(start_vertex);
  frontier.insert(start_vertex);

  for (int ring = 0; ring < k; ++ring) {
    std::unordered_set<int> next_frontier;

    for (int u : frontier) {
      for (int neighbor : adjacency[u]) {
        if (seen.find(neighbor) == seen.end()) {
          next_frontier.insert(neighbor);
        }
      }
    }

    if (next_frontier.empty()) {
      break;
    }

    seen.insert(next_frontier.begin(), next_frontier.end());
    frontier = std::move(next_frontier);
  }

  return seen;
}

Eigen::VectorXd GaussianCurvature::compute_kring_smoothed(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh, int ring_k) {

  // First compute raw curvature
  Eigen::VectorXd raw_curvature = compute_angle_deficit(mesh);

  // Build adjacency
  auto adjacency = build_vertex_adjacency(mesh);

  // Smooth by averaging over k-ring neighborhood
  int n = static_cast<int>(mesh->vertices_.size());
  Eigen::VectorXd smoothed_curvature(n);

  for (int i = 0; i < n; ++i) {
    auto neighbors = get_kring_neighbors(adjacency, i, ring_k);

    double sum = 0.0;
    for (int neighbor_id : neighbors) {
      sum += raw_curvature(neighbor_id);
    }

    smoothed_curvature(i) = sum / static_cast<double>(neighbors.size());
  }

  return smoothed_curvature;
}

} // namespace crobot::geometry
