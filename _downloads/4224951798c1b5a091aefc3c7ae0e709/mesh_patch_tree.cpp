#include "crobot/patch_tree/mesh_patch_tree.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Eigenvalues>
#include <open3d/geometry/TriangleMesh.h>

#include "crobot/geometry/mesh_utils.hpp"

namespace crobot::patch_tree {
using geometry::MeshUtils;
namespace {
using Vec      = Eigen::Vector3d;
using Triangle = std::vector<Vec>;

PatchGenerationOptions normal_only_options() {
  PatchGenerationOptions options;
  options.max_torque_variation = 0;
  return options;
}

std::vector<Triangle> split_triangle(const Triangle& t) {
  const Vec a = (t[0] + t[1]) / 2;
  const Vec b = (t[0] + t[2]) / 2;
  const Vec c = (t[1] + t[2]) / 2;
  return {{t[0], a, b}, {a, t[1], c}, {a, b, c}, {b, c, t[2]}};
}

int direction_bin(const Vec& direction, const std::vector<Triangle>& bins) {
  for (size_t i = 0; i < bins.size(); ++i)
    if (MeshUtils::ray_intersects_triangle(
            direction, bins[i][0], bins[i][1], bins[i][2]))
      return static_cast<int>(i);
  throw std::runtime_error(
      "Normal could not be assigned to a Gauss-map triangle");
}

bool exceeds(double value, double limit) {
  return value > limit + 1e-10 * std::max(1.0, std::abs(limit));
}
} // namespace

MeshPatchTree::MeshPatchTree(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh, const Vec& center)
  : MeshPatchTree(std::move(mesh), center, normal_only_options()) {
}

MeshPatchTree::MeshPatchTree(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
    const Vec& center,
    const PatchGenerationOptions& options,
    std::shared_ptr<const MonotoneDecomposition> monotone,
    std::vector<int> analysis_face_ids)
  : mesh_(std::move(mesh)),
    center_(center),
    options_(options),
    monotone_(std::move(monotone)),
    analysis_face_ids_(std::move(analysis_face_ids)) {
  if (!mesh_ || !center.allFinite())
    throw std::invalid_argument("Invalid patch-tree mesh or origin");
  if (!std::isfinite(options.max_total_curvature)
      || options.max_total_curvature <= 0
      || !std::isfinite(options.max_torque_variation)
      || options.max_torque_variation < 0
      || !std::isfinite(options.max_normal_angle)
      || options.max_normal_angle <= 0 || options.max_normal_angle > M_PI
      || options.max_depth < 3 || options.max_refinement_rounds < 0
      || options.max_prepared_faces == 0)
    throw std::invalid_argument("Invalid patch generation limits");
  selected_.resize(mesh_->triangles_.size(), false);
  prepare_geometry();
  if (options_.split_by_curvature && !monotone_)
    monotone_ = std::make_shared<MonotoneDecomposition>(
        decompose_monotone(*mesh_, options_.monotone));
  if (analysis_face_ids_.empty()) {
    analysis_face_ids_.resize(mesh_->triangles_.size());
    std::iota(analysis_face_ids_.begin(), analysis_face_ids_.end(), 0);
  }
  if (analysis_face_ids_.size() != mesh_->triangles_.size())
    throw std::invalid_argument("Analysis face correspondence size mismatch");
  if (monotone_) {
    for (int f : analysis_face_ids_)
      if (f < 0 || f >= static_cast<int>(monotone_->uncertain.size()))
        throw std::invalid_argument(
            "Analysis face correspondence out of range");
    for (const auto& cut : monotone_->cuts) {
      if (cut.face_regions.size() != monotone_->uncertain.size())
        throw std::invalid_argument("Monotone partition size mismatch");
      for (int id : cut.face_regions)
        if (id < 0 || id >= static_cast<int>(cut.regions.size()))
          throw std::invalid_argument("Monotone region ID out of range");
    }
  }
}

void MeshPatchTree::prepare_geometry() {
  // Validate before asking Open3D to dereference any triangle indices.
  for (const auto& triangle : mesh_->triangles_)
    for (int j = 0; j < 3; ++j)
      if (triangle[j] < 0
          || triangle[j] >= static_cast<int>(mesh_->vertices_.size()))
        throw std::invalid_argument("Triangle vertex index out of range");
  if (!mesh_->HasVertexNormals())
    mesh_->ComputeVertexNormals();
  for (const auto& p : mesh_->vertices_)
    if (!p.allFinite())
      throw std::invalid_argument("Non-finite mesh vertex");
  if (mesh_->vertex_normals_.size() != mesh_->vertices_.size())
    throw std::invalid_argument("Missing mesh normals");
  face_neighbors_.resize(mesh_->triangles_.size());
  face_normals_.resize(mesh_->triangles_.size());
  face_centers_.resize(mesh_->triangles_.size());
  face_curvatures_.resize(mesh_->triangles_.size());
  std::map<std::pair<int, int>, std::vector<int>> edges;
  for (size_t i = 0; i < mesh_->triangles_.size(); ++i) {
    const auto& t = mesh_->triangles_[i];
    for (int j = 0; j < 3; ++j) {
      if (t[j] < 0 || t[j] >= static_cast<int>(mesh_->vertices_.size()))
        throw std::invalid_argument("Triangle vertex index out of range");
      const auto& n = mesh_->vertex_normals_[t[j]];
      if (!n.allFinite() || std::abs(n.norm() - 1) > 1e-5)
        throw std::invalid_argument(
            "Patch generation requires finite unit normals; prepare the mesh "
            "first");
      edges[std::minmax(t[j], t[(j + 1) % 3])].push_back(i);
    }
    const Vec a = mesh_->vertices_[t[0]], b = mesh_->vertices_[t[1]],
              c = mesh_->vertices_[t[2]];
    if ((b - a).cross(c - a).squaredNorm() == 0)
      throw std::invalid_argument("Degenerate face; prepare the mesh first");
    const Vec n1 = mesh_->vertex_normals_[t[0]],
              n2 = mesh_->vertex_normals_[t[1]],
              n3 = mesh_->vertex_normals_[t[2]];
    Vec average  = n1 + n2 + n3;
    if (average.norm() < 1e-12)
      average = (b - a).cross(c - a);
    face_normals_[i]    = average.normalized();
    face_centers_[i]    = (a + b + c) / 3;
    face_curvatures_[i] = MeshUtils::compute_solid_angle(n1, n2, n3);
    if (!std::isfinite(face_curvatures_[i]))
      throw std::invalid_argument("Non-finite face curvature");
  }
  for (const auto& entry : edges) {
    const auto& faces = entry.second;
    if (faces.size() != 2)
      continue;
    int starts[2] = {-1, -1};
    for (int i = 0; i < 2; ++i) {
      const auto& t = mesh_->triangles_[faces[i]];
      for (int j = 0; j < 3; ++j) {
        const std::pair<int, int> key = std::minmax(t[j], t[(j + 1) % 3]);
        if (key == entry.first)
          starts[i] = t[j];
      }
    }
    if (starts[0] == starts[1])
      continue;
    face_neighbors_[faces[0]].push_back(faces[1]);
    face_neighbors_[faces[1]].push_back(faces[0]);
    face_edges_.emplace_back(faces[0], faces[1]);
  }
  for (auto& neighbors : face_neighbors_) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(
        std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }
}

void MeshPatchTree::add_triangle(int tid) {
  if (tid < 0 || tid >= static_cast<int>(selected_.size()))
    throw std::out_of_range("Triangle ID out of range");
  if (selected_[tid])
    throw std::invalid_argument("Triangle already added to tree");
  selected_[tid] = true;
  input_ids_.push_back(tid);
  dirty_ = true;
  planning_tree_.reset();
}

std::vector<std::vector<int>> MeshPatchTree::connected_groups(
    const std::vector<int>& ids, const std::vector<int>& labels) const {
  std::unordered_map<int, int> remaining;
  for (size_t i = 0; i < ids.size(); ++i)
    remaining.emplace(ids[i], labels[i]);
  std::vector<std::vector<int>> result;
  for (int seed : ids) {
    auto found = remaining.find(seed);
    if (found == remaining.end())
      continue;
    const int label = found->second;
    remaining.erase(found);
    std::vector<int> group{seed};
    for (size_t q = 0; q < group.size(); ++q) {
      for (int next : face_neighbors_[group[q]]) {
        found = remaining.find(next);
        if (found != remaining.end() && found->second == label) {
          group.push_back(next);
          remaining.erase(found);
        }
      }
    }
    std::sort(group.begin(), group.end());
    result.push_back(std::move(group));
  }
  return result;
}

MeshPatchTree::Bounds MeshPatchTree::measure(
    const std::vector<int>& ids) const {
  Bounds result;
  if (ids.empty())
    return result;
  std::vector<int> vertices;
  for (int id : ids) {
    result.curvature += face_curvatures_[id];
    const auto& t = mesh_->triangles_[id];
    for (int j = 0; j < 3; ++j)
      vertices.push_back(t[j]);
  }
  std::sort(vertices.begin(), vertices.end());
  vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
  Vec normal_sum = Vec::Zero(), mean = Vec::Zero();
  std::vector<Vec> torques;
  torques.reserve(vertices.size());
  for (int v : vertices) {
    normal_sum += mesh_->vertex_normals_[v];
    torques.push_back(
        (mesh_->vertices_[v] - center_).cross(mesh_->vertex_normals_[v]));
    mean += torques.back();
  }
  if (normal_sum.norm() < 1e-12)
    result.normal_angle = M_PI;
  else {
    const Vec axis = normal_sum.normalized();
    double radius  = 0;
    for (int v : vertices) {
      const auto& n = mesh_->vertex_normals_[v];
      radius = std::max(radius, std::atan2(axis.cross(n).norm(), axis.dot(n)));
    }
    result.normal_angle = std::min(M_PI, 2 * radius);
  }
  mean /= torques.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& t : torques)
    covariance.noalias() += (t - mean) * (t - mean).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("Torque PCA failed");
  for (int i = 0; i < 3; ++i) {
    Vec axis = solver.eigenvectors().col(2 - i);
    Eigen::Index maximum;
    axis.cwiseAbs().maxCoeff(&maximum);
    if (axis[maximum] < 0)
      axis = -axis;
    result.torque_axes.col(i) = axis;
  }
  Vec lower = Vec::Constant(std::numeric_limits<double>::infinity());
  Vec upper = -lower;
  for (const auto& t : torques) {
    const Vec local = result.torque_axes.transpose() * (t - mean);
    lower           = lower.cwiseMin(local);
    upper           = upper.cwiseMax(local);
  }
  result.torque_extent = upper - lower;
  result.torque_center = mean + result.torque_axes * ((lower + upper) / 2);
  result.torque_span   = result.torque_extent.norm();
  return result;
}

std::vector<int> MeshPatchTree::spatial_bisection(
    const std::vector<int>& ids) const {
  Vec lower = face_centers_[ids.front()], upper = lower;
  for (int id : ids) {
    lower = lower.cwiseMin(face_centers_[id]);
    upper = upper.cwiseMax(face_centers_[id]);
  }
  Eigen::Index axis;
  (upper - lower).maxCoeff(&axis);
  auto sorted = ids;
  std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
    const double x = face_centers_[a][axis], y = face_centers_[b][axis];
    return x != y ? x < y : a < b;
  });
  std::set<int> first(sorted.begin(), sorted.begin() + sorted.size() / 2);
  std::vector<int> labels;
  for (int id : ids)
    labels.push_back(first.count(id) ? 0 : 1);
  return labels;
}

std::vector<std::vector<int>> MeshPatchTree::connected_bisection(
    const std::vector<int>& ids) const {
  // A spatial median usually provides a compact, connected cut. If a handle
  // or cavity disconnects either side, cut a spanning-tree edge instead.
  auto groups = connected_groups(ids, spatial_bisection(ids));
  if (groups.size() == 2)
    return groups;

  std::unordered_map<int, std::pair<int, size_t>> state;
  for (int id : ids)
    state.emplace(id, std::make_pair(-2, size_t{1}));
  Vec lower = face_centers_[ids.front()], upper = lower;
  for (int id : ids) {
    lower = lower.cwiseMin(face_centers_[id]);
    upper = upper.cwiseMax(face_centers_[id]);
  }
  Eigen::Index axis;
  (upper - lower).maxCoeff(&axis);
  const int seed = *std::min_element(ids.begin(), ids.end(), [&](int a, int b) {
    const double x = face_centers_[a][axis], y = face_centers_[b][axis];
    return x != y ? x < y : a < b;
  });
  state.at(seed).first = -1;
  std::vector<int> order{seed};
  for (size_t cursor = 0; cursor < order.size(); ++cursor) {
    const int face = order[cursor];
    for (int next : face_neighbors_[face]) {
      auto found = state.find(next);
      if (found != state.end() && found->second.first == -2) {
        found->second.first = face;
        order.push_back(next);
      }
    }
  }
  if (order.size() != ids.size())
    throw std::logic_error("Coarse bisection requires a connected patch");
  int cut                = -1;
  size_t best_difference = ids.size();
  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    const auto& node = state.at(*it);
    if (node.first < 0)
      continue;
    const size_t complement = ids.size() - node.second;
    const size_t difference = node.second > complement
                                  ? node.second - complement
                                  : complement - node.second;
    if (difference < best_difference
        || (difference == best_difference && *it < cut)) {
      best_difference = difference;
      cut             = *it;
    }
    state.at(node.first).second += node.second;
  }
  if (cut < 0)
    throw std::logic_error("Cannot bisect a singleton patch");
  std::set<int> subtree{cut};
  order.assign(1, cut);
  for (size_t cursor = 0; cursor < order.size(); ++cursor)
    for (int next : face_neighbors_[order[cursor]]) {
      auto found = state.find(next);
      if (found != state.end() && found->second.first == order[cursor]) {
        subtree.insert(next);
        order.push_back(next);
      }
    }
  groups.assign(2, {});
  for (int id : ids)
    groups[subtree.count(id) ? 0 : 1].push_back(id);
  return groups;
}

int MeshPatchTree::add_node(
    int parent,
    const std::vector<int>& ids,
    int type,
    const Triangle& triangle) const {
  const int id = nodes_.size();
  auto patch   = std::make_shared<MeshPatch>(mesh_, triangle);
  for (int face : ids)
    patch->add_triangle(face, face_normals_[face], face_curvatures_[face]);
  auto& node = nodes_.emplace(id, Node(id, parent, patch)).first->second;
  node.data.parent_id = parent;
  node.data.type      = type;
  node.data.face_ids  = ids;
  node.data.stage     = parent < 0  ? PatchTree::Stage::Object
                        : type == 2 ? PatchTree::Stage::Normal
                        : type == 3 ? PatchTree::Stage::Torque
                                    : PatchTree::Stage::Spatial;
  if (parent >= 0) {
    node.data.monotone_region_id = nodes_.at(parent).data.monotone_region_id;
    node.data.curvature_class    = nodes_.at(parent).data.curvature_class;
  }
  if (monotone_ && !ids.empty()) {
    double area = 0, uncertain = 0;
    for (int f : ids) {
      const auto& t = mesh_->triangles_[f];
      const double a
          = (mesh_->vertices_[t[1]] - mesh_->vertices_[t[0]])
                .cross(mesh_->vertices_[t[2]] - mesh_->vertices_[t[0]])
                .norm();
      area += a;
      if (monotone_->uncertain[analysis_face_ids_[f]])
        uncertain += a;
    }
    node.data.uncertain_area_fraction = uncertain / area;
  }
  const auto bounds         = measure(ids);
  node.data.total_curvature = bounds.curvature;
  node.data.max_normal_angle
      = parent < 0
            ? bounds.normal_angle
            : std::min(
                bounds.normal_angle, nodes_.at(parent).data.max_normal_angle);
  node.data.torque_span = bounds.torque_span;
  node.data.curvature_status
      = exceeds(bounds.curvature, options_.max_total_curvature)
            ? PatchTree::BoundStatus::NotEvaluated
            : PatchTree::BoundStatus::Satisfied;
  node.data.normal_status
      = exceeds(node.data.max_normal_angle, options_.max_normal_angle)
            ? PatchTree::BoundStatus::NotEvaluated
            : PatchTree::BoundStatus::Satisfied;
  node.data.torque_status
      = options_.max_torque_variation == 0
            ? PatchTree::BoundStatus::NotEvaluated
        : exceeds(bounds.torque_span, options_.max_torque_variation)
            ? PatchTree::BoundStatus::NotEvaluated
            : PatchTree::BoundStatus::Satisfied;
  if (type == 2)
    node.data.vertices = MeshUtils::enclosing_normal_directions(*mesh_, ids);
  if (type == 3) {
    for (int corner = 0; corner < 8; ++corner) {
      Vec local;
      for (int axis = 0; axis < 3; ++axis)
        local[axis]
            = (corner & (1 << axis) ? 0.5 : -0.5) * bounds.torque_extent[axis];
      node.data.vertices.push_back(
          bounds.torque_center + bounds.torque_axes * local);
    }
  }
  if (parent >= 0)
    nodes_.at(parent).children_ids.push_back(id);
  return id;
}

void MeshPatchTree::set_leaf_status(int id, int depth) const {
  auto& node            = nodes_.at(id);
  const auto unresolved = node.data.face_ids.size() == 1
                              ? PatchTree::BoundStatus::Indivisible
                              : PatchTree::BoundStatus::RefinementLimit;
  (void)depth;
  if (exceeds(node.data.total_curvature, options_.max_total_curvature))
    node.data.curvature_status = unresolved;
  if (exceeds(node.data.max_normal_angle, options_.max_normal_angle))
    node.data.normal_status = unresolved;
  if (options_.max_torque_variation > 0
      && exceeds(node.data.torque_span, options_.max_torque_variation))
    node.data.torque_status = unresolved;
}

void MeshPatchTree::refine_normal(
    int id, int depth, const Triangle& triangle) const {
  auto& node      = nodes_.at(id);
  const auto& ids = node.data.face_ids;
  const bool needs_normal
      = exceeds(node.data.total_curvature, options_.max_total_curvature)
        || exceeds(node.data.max_normal_angle, options_.max_normal_angle);
  if (needs_normal && ids.size() > 1 && depth < options_.max_depth) {
    const auto bins = split_triangle(triangle);
    std::vector<int> labels;
    for (int face : ids)
      labels.push_back(direction_bin(face_normals_[face], bins));
    auto groups         = connected_groups(ids, labels);
    const bool fallback = groups.size() == 1;
    if (fallback)
      groups = connected_groups(ids, spatial_bisection(ids));
    for (const auto& group : groups) {
      const auto& child_triangle
          = fallback ? triangle
                     : bins[direction_bin(face_normals_[group.front()], bins)];
      const int child = add_node(id, group, 2, child_triangle);
      refine_normal(child, depth + 1, child_triangle);
    }
  }
  else if (
      options_.max_torque_variation > 0
      && exceeds(node.data.torque_span, options_.max_torque_variation)
      && ids.size() > 1 && depth < options_.max_depth) {
    // Keep a normal node above every torque subtree for the legacy descriptor
    // API.
    const int child = add_node(id, ids, 3, triangle);
    refine_torque(child, depth + 1);
  }
  else
    set_leaf_status(id, depth);
}

void MeshPatchTree::refine_torque(int id, int depth) const {
  auto& node      = nodes_.at(id);
  const auto& ids = node.data.face_ids;
  if (exceeds(node.data.torque_span, options_.max_torque_variation)
      && ids.size() > 1 && depth < options_.max_depth) {
    const auto bounds = measure(ids);
    std::vector<int> labels;
    for (int face : ids) {
      const auto& triangle = mesh_->triangles_[face];
      Vec mean             = Vec::Zero();
      for (int j = 0; j < 3; ++j) {
        const int v = triangle[j];
        mean += (mesh_->vertices_[v] - center_).cross(mesh_->vertex_normals_[v])
                / 3;
      }
      const Vec local
          = bounds.torque_axes.transpose() * (mean - bounds.torque_center);
      labels.push_back((local[0] > 0 ? 1 : 0) + (local[1] > 0 ? 2 : 0));
    }
    auto groups = connected_groups(ids, labels);
    if (groups.size() == 1)
      groups = connected_groups(ids, spatial_bisection(ids));
    for (const auto& group : groups) {
      const int child = add_node(id, group, 3, {});
      refine_torque(child, depth + 1);
    }
  }
  else
    set_leaf_status(id, depth);
}

void MeshPatchTree::finalize() const {
  if (!dirty_)
    return;
  nodes_.clear();
  planning_tree_.reset();
  auto sorted_ids = input_ids_;
  std::sort(sorted_ids.begin(), sorted_ids.end());
  add_node(-1, sorted_ids, 1, {});
  if (!input_ids_.empty()) {
    const auto ico = open3d::geometry::TriangleMesh::CreateIcosahedron();
    std::vector<Triangle> bins;
    for (const auto& t : ico->triangles_)
      bins.push_back(
          {ico->vertices_[t[0]], ico->vertices_[t[1]], ico->vertices_[t[2]]});
    const auto normal_regions
        = [&](int parent, const std::vector<int>& region, int depth) {
            if (depth >= options_.max_depth) {
              set_leaf_status(parent, depth);
              return;
            }
            std::vector<int> labels;
            for (int face : region)
              labels.push_back(direction_bin(face_normals_[face], bins));
            for (const auto& group : connected_groups(region, labels)) {
              const auto& triangle
                  = bins[direction_bin(face_normals_[group.front()], bins)];
              const int child = add_node(parent, group, 2, triangle);
              refine_normal(child, depth + 1, triangle);
            }
          };
    std::function<void(int, const std::vector<int>&, int)> spatial;
    spatial = [&](int parent, const std::vector<int>& faces, int depth) {
      if (options_.max_coarse_faces > 0
          && faces.size() > options_.max_coarse_faces
          && depth + 1 < options_.max_depth) {
        for (const auto& group : connected_bisection(faces)) {
          const int child = add_node(parent, group, 1, {});
          spatial(child, group, depth + 1);
        }
      }
      else
        normal_regions(parent, faces, depth);
    };
    std::function<void(int, const std::vector<int>&, int, int)>
        monotone_regions;
    monotone_regions
        = [&](int parent, const std::vector<int>& faces, int cut, int depth) {
            if (depth >= options_.max_depth) {
              set_leaf_status(parent, depth);
              return;
            }
            std::vector<int> labels;
            for (int face : faces)
              labels.push_back(
                  monotone_->cuts[cut].face_regions[analysis_face_ids_[face]]);
            for (const auto& group : connected_groups(faces, labels)) {
              const int region
                  = monotone_->cuts[cut]
                        .face_regions[analysis_face_ids_[group.front()]];
              const int child = add_node(parent, group, 1, {});
              auto& data      = nodes_.at(child).data;
              data.stage      = cut == 2   ? PatchTree::Stage::Monotone
                                : cut == 1 ? PatchTree::Stage::DetailGrouping
                                           : PatchTree::Stage::CurvatureRegion;
              data.curvature_class
                  = static_cast<int>(monotone_->cuts[cut].regions[region].kind);
              if (cut == 2)
                data.monotone_region_id = region;
              if (cut > 0)
                monotone_regions(child, group, cut - 1, depth + 1);
              else
                spatial(child, group, depth + 1);
            }
          };
    for (const auto& component :
         connected_groups(sorted_ids, std::vector<int>(sorted_ids.size(), 0))) {
      const int child             = add_node(0, component, 1, {});
      nodes_.at(child).data.stage = PatchTree::Stage::Component;
      if (options_.split_by_curvature)
        monotone_regions(child, component, 2, 1);
      else
        spatial(child, component, 1);
    }
  }
  dirty_ = false;
}

void MeshPatchTree::ensure_built() const {
  if (dirty_)
    finalize();
}
const std::map<int, MeshPatchTree::Node>& MeshPatchTree::get_nodes() const {
  ensure_built();
  return nodes_;
}
std::vector<int> MeshPatchTree::get_face_leaf_ids() const {
  ensure_built();
  std::vector<int> labels(mesh_->triangles_.size(), -1);
  for (const auto& [id, node] : nodes_)
    if (node.children_ids.empty())
      for (int face : node.data.face_ids)
        labels[face] = id;
  return labels;
}
PatchTree::Ptr MeshPatchTree::get_planning_tree(
    const std::vector<std::vector<int>>& face_source_ids) const {
  ensure_built();
  if (!face_source_ids.empty()
      && face_source_ids.size() != mesh_->triangles_.size())
    throw std::invalid_argument(
        "Source face mapping size differs from prepared mesh");
  if (!planning_tree_ || !face_source_ids.empty()) {
    std::vector<PatchTree::NodeData> records;
    for (const auto& [id, node] : nodes_) {
      (void)id;
      records.push_back(node.data);
      if (!face_source_ids.empty())
        for (int face : node.data.face_ids)
          records.back().source_face_ids.insert(
              records.back().source_face_ids.end(),
              face_source_ids[face].begin(),
              face_source_ids[face].end());
    }
    const auto labels = get_face_leaf_ids();
    std::set<PatchTree::PatchEdge> edges;
    for (const auto& edge : face_edges_)
      if (labels[edge.first] >= 0 && labels[edge.second] >= 0
          && labels[edge.first] != labels[edge.second])
        edges.insert(std::minmax(labels[edge.first], labels[edge.second]));
    planning_tree_ = std::make_shared<PatchTree>(
        std::move(records),
        std::vector<PatchTree::PatchEdge>(edges.begin(), edges.end()));
  }
  return planning_tree_;
}

std::vector<std::pair<
    std::shared_ptr<open3d::geometry::TriangleMesh>,
    std::shared_ptr<open3d::geometry::TriangleMesh>>>
MeshPatchTree::gather_patches(
    bool include_visualization, bool compact_vertices) {
  finalize();
  std::vector<std::pair<
      std::shared_ptr<open3d::geometry::TriangleMesh>,
      std::shared_ptr<open3d::geometry::TriangleMesh>>>
      result;
  for (const auto& [id, node] : nodes_) {
    (void)id;
    if (!node.children_ids.empty() || node.data.face_ids.empty())
      continue;
    auto surface = MeshUtils::extract_triangles(
        *mesh_, node.data.face_ids, compact_vertices);
    std::shared_ptr<open3d::geometry::TriangleMesh> visualization;
    if (include_visualization)
      visualization = node.patch->convert_to_mesh().second;
    result.emplace_back(surface, visualization);
  }
  return result;
}

double MeshPatchTree::compute_total_curvature() const {
  ensure_built();
  return nodes_.at(0).patch->get_total_curvature();
}
void MeshPatchTree::show() const {
  ensure_built();
  show_recursive(0, 0);
}
void MeshPatchTree::show_recursive(int id, int depth) const {
  const auto& node = nodes_.at(id);
  std::cout << std::string(2 * depth, ' ') << "Node " << id << " (faces "
            << node.data.face_ids.size() << ", curvature "
            << node.data.total_curvature << ", torque span "
            << node.data.torque_span << ")\n";
  for (int child : node.children_ids)
    show_recursive(child, depth + 1);
}

} // namespace crobot::patch_tree
