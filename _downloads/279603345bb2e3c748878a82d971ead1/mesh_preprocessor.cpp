#include "crobot/geometry/mesh_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <open3d/geometry/IntersectionTest.h>
#include <open3d/geometry/TriangleMesh.h>

namespace crobot::geometry {
namespace {

using Edge = std::array<int, 2>;
struct EdgeFace
{
  int face;
  int direction;
};
using Edges = std::map<Edge, std::vector<EdgeFace>>;

Edge edge_key(int a, int b) {
  return {std::min(a, b), std::max(a, b)};
}

Edges collect_edges(const std::vector<Eigen::Vector3i>& faces) {
  Edges edges;
  for (std::size_t i = 0; i < faces.size(); ++i) {
    for (int corner = 0; corner < 3; ++corner) {
      const int a = faces[i][corner];
      const int b = faces[i][(corner + 1) % 3];
      edges[edge_key(a, b)].push_back({static_cast<int>(i), a < b ? 1 : -1});
    }
  }
  return edges;
}

class DisjointSet {
public:
  explicit DisjointSet(std::size_t size) : parent_(size), rank_(size, 0) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  int find(int value) {
    if (parent_[value] != value)
      parent_[value] = find(parent_[value]);
    return parent_[value];
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b)
      return;
    if (rank_[a] < rank_[b])
      std::swap(a, b);
    parent_[b] = a;
    if (rank_[a] == rank_[b])
      ++rank_[a];
  }

private:
  std::vector<int> parent_;
  std::vector<int> rank_;
};

void flip_face(open3d::geometry::TriangleMesh& mesh, int id) {
  std::swap(mesh.triangles_[id][1], mesh.triangles_[id][2]);
  if (!mesh.triangle_uvs_.empty())
    std::swap(mesh.triangle_uvs_[3 * id + 1], mesh.triangle_uvs_[3 * id + 2]);
}

// Cross-shell intersection queries use a median-split triangle BVH. Open3D's
// complete-mesh IsIntersecting implementation compares every face pair.
class ShellBvh {
public:
  ShellBvh(
      const open3d::geometry::TriangleMesh& mesh, const std::vector<int>& faces)
    : mesh_(mesh), faces_(faces) {
    build(0, static_cast<int>(faces_.size()));
  }

  bool intersects(const ShellBvh& other) const {
    std::vector<std::pair<int, int>> pending{{0, 0}};
    while (!pending.empty()) {
      const auto pair = pending.back();
      pending.pop_back();
      const auto& a = nodes_[pair.first];
      const auto& b = other.nodes_[pair.second];
      if ((a.high.array() < b.low.array()).any()
          || (b.high.array() < a.low.array()).any())
        continue;
      if (a.left < 0 && b.left < 0) {
        for (int i = a.begin; i < a.end; ++i)
          for (int j = b.begin; j < b.end; ++j) {
            const auto& first   = mesh_.triangles_[faces_[i]];
            const auto& second  = mesh_.triangles_[other.faces_[j]];
            Eigen::Vector3d low = mesh_.vertices_[first[0]], high = low;
            for (int k = 0; k < 3; ++k) {
              low = low.cwiseMin(mesh_.vertices_[first[k]])
                        .cwiseMin(mesh_.vertices_[second[k]]);
              high = high.cwiseMax(mesh_.vertices_[first[k]])
                         .cwiseMax(mesh_.vertices_[second[k]]);
            }
            const Eigen::Vector3d anchor = low + (high - low) * 0.5;
            const double scale           = (high - low).maxCoeff();
            if (open3d::geometry::IntersectionTest::TriangleTriangle3d(
                    (mesh_.vertices_[first[0]] - anchor) / scale,
                    (mesh_.vertices_[first[1]] - anchor) / scale,
                    (mesh_.vertices_[first[2]] - anchor) / scale,
                    (mesh_.vertices_[second[0]] - anchor) / scale,
                    (mesh_.vertices_[second[1]] - anchor) / scale,
                    (mesh_.vertices_[second[2]] - anchor) / scale))
              return true;
          }
      }
      else if (
          b.left < 0 || (a.left >= 0 && a.end - a.begin >= b.end - b.begin)) {
        pending.emplace_back(a.left, pair.second);
        pending.emplace_back(a.right, pair.second);
      }
      else {
        pending.emplace_back(pair.first, b.left);
        pending.emplace_back(pair.first, b.right);
      }
    }
    return false;
  }

private:
  struct Node
  {
    Eigen::Vector3d low, high;
    int begin, end, left = -1, right = -1;
  };

  int build(int begin, int end) {
    Node node;
    node.begin = begin;
    node.end   = end;
    node.low   = mesh_.vertices_[mesh_.triangles_[faces_[begin]][0]];
    node.high  = node.low;
    for (int i = begin; i < end; ++i)
      for (int k = 0; k < 3; ++k) {
        const auto& p = mesh_.vertices_[mesh_.triangles_[faces_[i]][k]];
        node.low      = node.low.cwiseMin(p);
        node.high     = node.high.cwiseMax(p);
      }
    const int id = static_cast<int>(nodes_.size());
    nodes_.push_back(node);
    if (end - begin > 8) {
      Eigen::Index axis;
      (node.high - node.low).maxCoeff(&axis);
      const int middle  = begin + (end - begin) / 2;
      const auto center = [&](int face) {
        const auto& t = mesh_.triangles_[face];
        return mesh_.vertices_[t[0]][axis] / 3.0
               + mesh_.vertices_[t[1]][axis] / 3.0
               + mesh_.vertices_[t[2]][axis] / 3.0;
      };
      std::nth_element(
          faces_.begin() + begin,
          faces_.begin() + middle,
          faces_.begin() + end,
          [&](int a, int b) {
            const double x = center(a), y = center(b);
            return x != y ? x < y : a < b;
          });
      const int left = build(begin, middle), right = build(middle, end);
      nodes_[id].left  = left;
      nodes_[id].right = right;
    }
    return id;
  }
  const open3d::geometry::TriangleMesh& mesh_;
  std::vector<int> faces_;
  std::vector<Node> nodes_;
};

struct ClosedShell
{
  int component;
  Eigen::Vector3d low, high;
  long double six_volume = 0;
  bool ambiguous         = false;
  std::unique_ptr<ShellBvh> bvh;
};

// Returns 0 outside, 1 inside, and -1 for an unreliable/noninteger winding.
int contains_shell(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<int>& outer,
    const std::vector<int>& inner) {
  int classification = -1;
  for (int sample = 0; sample < 5; ++sample) {
    const auto& face = mesh.triangles_[inner[(inner.size() - 1) * sample / 4]];
    const Eigen::Vector3d point = mesh.vertices_[face[0]] / 3.0
                                  + mesh.vertices_[face[1]] / 3.0
                                  + mesh.vertices_[face[2]] / 3.0;
    long double angle = 0;
    for (int id : outer) {
      const auto& t     = mesh.triangles_[id];
      Eigen::Vector3d a = mesh.vertices_[t[0]] - point;
      Eigen::Vector3d b = mesh.vertices_[t[1]] - point;
      Eigen::Vector3d c = mesh.vertices_[t[2]] - point;
      const double an = a.stableNorm(), bn = b.stableNorm(),
                   cn = c.stableNorm();
      if (an == 0 || bn == 0 || cn == 0)
        return -1;
      a /= an;
      b /= bn;
      c /= cn;
      angle += 2.0
               * std::atan2(
                   a.dot(b.cross(c)), 1.0 + a.dot(b) + a.dot(c) + b.dot(c));
    }
    const double winding
        = std::abs(static_cast<double>(angle / (4 * std::acos(-1.0L))));
    const int current = winding < 1e-5                   ? 0
                        : std::abs(winding - 1.0) < 1e-5 ? 1
                                                         : -1;
    if (current < 0 || (sample > 0 && current != classification))
      return -1;
    classification = current;
  }
  return classification;
}

void orient_closed_shells(
    open3d::geometry::TriangleMesh& mesh,
    const std::vector<std::vector<int>>& components,
    const std::vector<bool>& closed,
    const std::vector<bool>& conflict,
    std::vector<int>& flip,
    const MeshPreparationOptions& options,
    MeshPreparationReport& report) {
  std::vector<ClosedShell> shells;
  for (std::size_t c = 0; c < components.size(); ++c) {
    if (!closed[c] || conflict[c])
      continue;
    const auto& ids = components[c];
    ClosedShell shell;
    shell.component = static_cast<int>(c);
    shell.low       = mesh.vertices_[mesh.triangles_[ids.front()][0]];
    shell.high      = shell.low;
    for (int id : ids)
      for (int k = 0; k < 3; ++k) {
        shell.low = shell.low.cwiseMin(mesh.vertices_[mesh.triangles_[id][k]]);
        shell.high
            = shell.high.cwiseMax(mesh.vertices_[mesh.triangles_[id][k]]);
      }
    const Eigen::Vector3d anchor = shell.low + (shell.high - shell.low) * 0.5;
    for (int id : ids) {
      const auto& t           = mesh.triangles_[id];
      const Eigen::Vector3d a = mesh.vertices_[t[0]] - anchor;
      const Eigen::Vector3d b = mesh.vertices_[t[1]] - anchor;
      const Eigen::Vector3d d = mesh.vertices_[t[2]] - anchor;
      shell.six_volume += static_cast<long double>(a.dot(b.cross(d)));
    }
    const long double extent    = (shell.high - shell.low).stableNorm();
    const long double tolerance = 128.0L
                                  * std::numeric_limits<double>::epsilon()
                                  * extent * extent * extent;
    shell.ambiguous = !std::isfinite(shell.six_volume)
                      || std::abs(shell.six_volume) <= tolerance;
    shells.push_back(std::move(shell));
  }
  std::vector<std::vector<bool>> inside(
      shells.size(), std::vector<bool>(shells.size(), false));
  for (std::size_t i = 0; i < shells.size(); ++i)
    for (std::size_t j = i + 1; j < shells.size(); ++j) {
      auto& a = shells[i];
      auto& b = shells[j];
      if ((a.high.array() < b.low.array()).any()
          || (b.high.array() < a.low.array()).any())
        continue;
      if (!a.bvh)
        a.bvh = std::make_unique<ShellBvh>(mesh, components[a.component]);
      if (!b.bvh)
        b.bvh = std::make_unique<ShellBvh>(mesh, components[b.component]);
      if (a.bvh->intersects(*b.bvh)) {
        a.ambiguous = b.ambiguous = true;
        ++report.intersecting_shell_pairs;
        continue;
      }
      if (!options.orient_nested_shells)
        continue;
      const auto classify = [&](std::size_t outer, std::size_t inner) {
        auto& out = shells[outer];
        auto& in  = shells[inner];
        if ((in.low.array() < out.low.array()).any()
            || (in.high.array() > out.high.array()).any())
          return;
        const int contained = contains_shell(
            mesh, components[out.component], components[in.component]);
        if (contained < 0)
          out.ambiguous = in.ambiguous = true;
        else
          inside[inner][outer] = contained == 1;
      };
      classify(i, j);
      classify(j, i);
    }
  // Do not determine material-side parity from an unreliable enclosing shell.
  bool changed;
  do {
    changed = false;
    for (std::size_t i = 0; i < shells.size(); ++i) {
      if (shells[i].ambiguous)
        continue;
      for (std::size_t j = 0; j < shells.size(); ++j)
        if (options.orient_nested_shells && shells[j].ambiguous
            && (inside[i][j]
                || ((shells[i].low.array() >= shells[j].low.array()).all()
                    && (shells[i].high.array() <= shells[j].high.array())
                           .all()))) {
          shells[i].ambiguous = true;
          changed             = true;
          break;
        }
    }
  } while (changed);
  for (std::size_t i = 0; i < shells.size(); ++i) {
    const auto& shell = shells[i];
    if (shell.ambiguous) {
      ++report.ambiguous_closed_components;
      report.ambiguous_closed_component_ids.push_back(shell.component);
      continue;
    }
    const int depth = std::count(inside[i].begin(), inside[i].end(), true);
    if (depth > 0)
      ++report.nested_closed_components;
    const bool inward = depth % 2 != 0;
    if ((shell.six_volume < 0) != inward)
      for (int id : components[shell.component]) {
        flip_face(mesh, id);
        flip[id] ^= 1;
      }
    if (inward)
      ++report.inward_oriented_components;
    else
      ++report.outward_oriented_components;
  }
}

void orient_and_inspect(
    open3d::geometry::TriangleMesh& mesh,
    const MeshPreparationOptions& options,
    MeshPreparationReport& report) {
  const auto edges = collect_edges(mesh.triangles_);
  const int count  = static_cast<int>(mesh.triangles_.size());
  DisjointSet components(count);
  std::vector<std::vector<std::pair<int, bool>>> constraints(count);
  for (const auto& entry : edges) {
    const auto& incident = entry.second;
    for (std::size_t i = 1; i < incident.size(); ++i)
      components.unite(incident.front().face, incident[i].face);
    if (incident.size() == 1)
      ++report.boundary_edges;
    if (incident.size() > 2) {
      ++report.nonmanifold_edges;
      report.nonmanifold_edge_vertex_ids.push_back(entry.first);
    }
    if (incident.size() != 2)
      continue;
    const bool different_flip = incident[0].direction == incident[1].direction;
    constraints[incident[0].face].push_back({incident[1].face, different_flip});
    constraints[incident[1].face].push_back({incident[0].face, different_flip});
  }
  for (int id = 0; id < count; ++id)
    if (components.find(id) == id)
      ++report.connected_components;

  std::vector<int> flip(count, -1);
  std::vector<int> orientation_component(count, -1);
  std::vector<std::vector<int>> orientation_faces;
  for (int seed = 0; seed < count; ++seed) {
    if (flip[seed] != -1)
      continue;
    const int component_id = static_cast<int>(orientation_faces.size());
    orientation_faces.push_back({});
    std::queue<int> pending;
    pending.push(seed);
    flip[seed] = 0;
    while (!pending.empty()) {
      const int id = pending.front();
      pending.pop();
      orientation_component[id] = component_id;
      orientation_faces.back().push_back(id);
      for (const auto& neighbor : constraints[id]) {
        if (flip[neighbor.first] != -1)
          continue;
        flip[neighbor.first] = flip[id] ^ static_cast<int>(neighbor.second);
        pending.push(neighbor.first);
      }
    }
  }
  report.orientation_components = orientation_faces.size();
  std::vector<bool> closed(orientation_faces.size(), true);
  std::vector<bool> conflict(orientation_faces.size(), false);
  for (const auto& entry : edges) {
    const auto& incident = entry.second;
    if (incident.size() != 2) {
      for (const auto& face : incident)
        closed[orientation_component[face.face]] = false;
      continue;
    }
    const bool expected = incident[0].direction == incident[1].direction;
    if ((flip[incident[0].face] ^ flip[incident[1].face]) != expected) {
      ++report.orientation_conflicts;
      report.orientation_conflict_edge_vertex_ids.push_back(entry.first);
      conflict[orientation_component[incident[0].face]] = true;
    }
  }

  if (options.orient_triangles) {
    for (int id = 0; id < count; ++id)
      if (flip[id])
        flip_face(mesh, id);
    orient_closed_shells(
        mesh, orientation_faces, closed, conflict, flip, options, report);
    report.flipped_faces = std::count(flip.begin(), flip.end(), 1);
  }
  for (const auto& entry : collect_edges(mesh.triangles_)) {
    const auto& incident = entry.second;
    if (incident.size() == 2 && incident[0].direction == incident[1].direction)
      ++report.inconsistent_manifold_edges;
  }
}

void compute_normals(
    open3d::geometry::TriangleMesh& mesh,
    const MeshPreparationOptions& options,
    MeshPreparationReport& report) {
  const auto edges        = collect_edges(mesh.triangles_);
  const std::size_t count = mesh.vertices_.size();
  std::vector<std::vector<int>> incident_faces(count);
  std::vector<std::vector<int>> neighbors(count);
  std::vector<double> strongest_area(count, 0.0);
  std::vector<Eigen::Vector3d> strongest_normal(
      count, Eigen::Vector3d::UnitZ());
  mesh.vertex_normals_.assign(count, Eigen::Vector3d::Zero());
  mesh.triangle_normals_.resize(mesh.triangles_.size());
  for (std::size_t id = 0; id < mesh.triangles_.size(); ++id) {
    const auto& triangle = mesh.triangles_[id];
    const Eigen::Vector3d cross
        = (mesh.vertices_[triangle[1]] - mesh.vertices_[triangle[0]])
              .cross(mesh.vertices_[triangle[2]] - mesh.vertices_[triangle[0]]);
    const double twice_area      = cross.stableNorm();
    const Eigen::Vector3d normal = cross / twice_area;
    mesh.triangle_normals_[id]   = normal;
    for (int k = 0; k < 3; ++k) {
      const int vertex = triangle[k];
      const Eigen::Vector3d u
          = (mesh.vertices_[triangle[(k + 1) % 3]] - mesh.vertices_[vertex])
                .stableNormalized();
      const Eigen::Vector3d v
          = (mesh.vertices_[triangle[(k + 2) % 3]] - mesh.vertices_[vertex])
                .stableNormalized();
      const double angle = std::atan2(u.cross(v).stableNorm(), u.dot(v));
      mesh.vertex_normals_[vertex] += angle * normal;
      incident_faces[vertex].push_back(static_cast<int>(id));
      if (twice_area > strongest_area[vertex]) {
        strongest_area[vertex]   = twice_area;
        strongest_normal[vertex] = normal;
      }
    }
  }

  const double cosine = std::cos(
      options.normal_smoothing_angle_degrees * std::acos(-1.0) / 180.0);
  std::vector<bool> feature(count, false);
  std::vector<int> boundary_count(count, 0);
  std::vector<bool> bad_edge(count, false);
  for (const auto& entry : edges) {
    const int a = entry.first[0], b = entry.first[1];
    const auto& faces = entry.second;
    neighbors[a].push_back(b);
    neighbors[b].push_back(a);
    if (faces.size() == 1) {
      ++boundary_count[a];
      ++boundary_count[b];
    }
    if (faces.size() > 2)
      bad_edge[a] = bad_edge[b] = true;
    if (faces.size() != 2
        || mesh.triangle_normals_[faces[0].face].dot(
               mesh.triangle_normals_[faces[1].face])
               < cosine)
      feature[a] = feature[b] = true;
  }
  // A manifold vertex has one connected incident face fan, with zero or two
  // boundary edges. Edge incidence alone misses shells touching at one point.
  for (std::size_t vertex = 0; vertex < count; ++vertex) {
    const auto& faces = incident_faces[vertex];
    if (faces.empty()) {
      ++report.unreferenced_vertices;
      continue;
    }
    bool nonmanifold
        = bad_edge[vertex]
          || (boundary_count[vertex] != 0 && boundary_count[vertex] != 2);
    std::set<int> visited;
    std::vector<int> pending{faces.front()};
    visited.insert(faces.front());
    while (!pending.empty()) {
      const int id = pending.back();
      pending.pop_back();
      const auto& triangle = mesh.triangles_[id];
      for (int k = 0; k < 3; ++k) {
        if (triangle[k] == static_cast<int>(vertex))
          continue;
        const auto& adjacent
            = edges.at(edge_key(static_cast<int>(vertex), triangle[k]));
        if (adjacent.size() != 2)
          continue;
        for (const auto& neighbor : adjacent)
          if (visited.insert(neighbor.face).second)
            pending.push_back(neighbor.face);
      }
    }
    nonmanifold = nonmanifold || visited.size() != faces.size();
    if (nonmanifold) {
      ++report.nonmanifold_vertices;
      report.nonmanifold_vertex_ids.push_back(static_cast<int>(vertex));
      feature[vertex] = true;
    }
  }
  for (std::size_t vertex = 0; vertex < count; ++vertex) {
    auto& normal        = mesh.vertex_normals_[vertex];
    const double length = normal.stableNorm();
    if (!normal.allFinite()
        || length <= 32.0 * std::numeric_limits<double>::epsilon()) {
      normal = strongest_normal[vertex];
      ++report.fallback_normal_vertices;
    }
    else
      normal /= length;
    if (feature[vertex])
      ++report.feature_vertices;
  }
  const auto original_normals = mesh.vertex_normals_;
  for (int iteration = 0; iteration < options.normal_smoothing_iterations;
       ++iteration) {
    auto filtered = mesh.vertex_normals_;
    for (std::size_t vertex = 0; vertex < count; ++vertex) {
      if (feature[vertex] || incident_faces[vertex].empty())
        continue;
      Eigen::Vector3d sum = 2.0 * mesh.vertex_normals_[vertex];
      for (int neighbor : neighbors[vertex]) {
        if (mesh.vertex_normals_[vertex].dot(mesh.vertex_normals_[neighbor])
            >= cosine)
          sum += mesh.vertex_normals_[neighbor];
      }
      const double length = sum.stableNorm();
      if (sum.allFinite()
          && length > 32.0 * std::numeric_limits<double>::epsilon())
        filtered[vertex] = sum / length;
    }
    mesh.vertex_normals_.swap(filtered);
  }
  for (std::size_t vertex = 0; vertex < count; ++vertex)
    if ((mesh.vertex_normals_[vertex] - original_normals[vertex]).norm()
        > 1e-12)
      ++report.smoothed_vertices;
  mesh.adjacency_list_.clear();
}

} // namespace

PreparedMesh prepare_mesh(
    const open3d::geometry::TriangleMesh& input,
    const MeshPreparationOptions& options) {
  if (options.normal_smoothing_iterations < 0
      || !std::isfinite(options.normal_smoothing_angle_degrees)
      || options.normal_smoothing_angle_degrees < 0.0
      || options.normal_smoothing_angle_degrees > 180.0
      || !std::isfinite(options.min_triangle_area)
      || options.min_triangle_area < 0.0)
    throw std::invalid_argument("Invalid mesh preparation options");
  if (input.vertices_.size()
          > static_cast<std::size_t>(std::numeric_limits<int>::max())
      || input.triangles_.size()
             > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("Mesh exceeds supported integer index range");
  for (std::size_t id = 0; id < input.vertices_.size(); ++id)
    if (!input.vertices_[id].allFinite())
      throw std::invalid_argument(
          "Nonfinite mesh vertex " + std::to_string(id));
  for (std::size_t id = 0; id < input.triangles_.size(); ++id)
    for (int k = 0; k < 3; ++k)
      if (input.triangles_[id][k] < 0
          || static_cast<std::size_t>(input.triangles_[id][k])
                 >= input.vertices_.size())
        throw std::invalid_argument(
            "Invalid vertex index in mesh face " + std::to_string(id));

  PreparedMesh result;
  result.mesh  = std::make_shared<open3d::geometry::TriangleMesh>(input);
  auto& mesh   = *result.mesh;
  auto& report = result.report;
  report.original_vertices = input.vertices_.size();
  report.original_faces    = input.triangles_.size();
  mesh.vertices_.clear();
  mesh.triangles_.clear();
  mesh.vertex_normals_.clear();
  mesh.triangle_normals_.clear();
  mesh.vertex_colors_.clear();
  mesh.triangle_uvs_.clear();
  mesh.triangle_material_ids_.clear();
  mesh.adjacency_list_.clear();
  std::map<std::array<double, 3>, int> unique_vertices;
  const bool has_colors = input.HasVertexColors();
  for (std::size_t id = 0; id < input.vertices_.size(); ++id) {
    const auto& vertex = input.vertices_[id];
    const std::array<double, 3> key{vertex[0], vertex[1], vertex[2]};
    const int next   = static_cast<int>(mesh.vertices_.size());
    const auto found = options.weld_exact_vertices
                           ? unique_vertices.emplace(key, next)
                           : std::make_pair(unique_vertices.end(), true);
    const int target = found.second ? next : found.first->second;
    result.original_vertex_to_prepared.push_back(target);
    if (found.second) {
      mesh.vertices_.push_back(vertex);
      if (has_colors)
        mesh.vertex_colors_.push_back(input.vertex_colors_[id]);
    }
  }
  report.welded_vertices = input.vertices_.size() - mesh.vertices_.size();
  result.original_face_to_prepared.assign(input.triangles_.size(), -1);
  result.original_face_disposition.assign(
      input.triangles_.size(), OriginalFaceDisposition::Degenerate);
  std::map<std::array<int, 3>, int> unique_faces;
  const bool has_uvs = input.HasTriangleUvs();
  const bool has_materials
      = input.triangle_material_ids_.size() == input.triangles_.size();
  for (std::size_t id = 0; id < input.triangles_.size(); ++id) {
    Eigen::Vector3i triangle;
    for (int k = 0; k < 3; ++k)
      triangle[k] = result.original_vertex_to_prepared[input.triangles_[id][k]];
    const Eigen::Vector3d cross
        = (mesh.vertices_[triangle[1]] - mesh.vertices_[triangle[0]])
              .cross(mesh.vertices_[triangle[2]] - mesh.vertices_[triangle[0]]);
    const double area = cross.stableNorm() * 0.5;
    if (!std::isfinite(area))
      throw std::invalid_argument(
          "Nonfinite area in mesh face " + std::to_string(id));
    if (area <= options.min_triangle_area) {
      ++report.degenerate_faces_removed;
      continue;
    }
    std::array<int, 3> key{triangle[0], triangle[1], triangle[2]};
    std::sort(key.begin(), key.end());
    const auto found
        = unique_faces.emplace(key, static_cast<int>(mesh.triangles_.size()));
    const int target                     = found.first->second;
    result.original_face_to_prepared[id] = target;
    if (!found.second) {
      result.original_face_disposition[id] = OriginalFaceDisposition::Duplicate;
      result.face_source_ids[target].push_back(static_cast<int>(id));
      ++report.duplicate_faces_removed;
      continue;
    }
    result.original_face_disposition[id] = OriginalFaceDisposition::Retained;
    result.face_source_ids.push_back({static_cast<int>(id)});
    mesh.triangles_.push_back(triangle);
    if (has_uvs)
      for (int k = 0; k < 3; ++k)
        mesh.triangle_uvs_.push_back(input.triangle_uvs_[3 * id + k]);
    if (has_materials)
      mesh.triangle_material_ids_.push_back(input.triangle_material_ids_[id]);
  }
  orient_and_inspect(mesh, options, report);
  compute_normals(mesh, options, report);
  result.original_face_to_prepared_faces.resize(input.triangles_.size());
  for (std::size_t id = 0; id < input.triangles_.size(); ++id)
    if (result.original_face_to_prepared[id] >= 0)
      result.original_face_to_prepared_faces[id].push_back(
          result.original_face_to_prepared[id]);
  report.prepared_vertices = mesh.vertices_.size();
  report.prepared_faces    = mesh.triangles_.size();
  return result;
}

} // namespace crobot::geometry
