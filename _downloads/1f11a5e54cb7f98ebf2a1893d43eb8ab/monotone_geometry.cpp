#include <algorithm>
#include <numeric>
#include <stdexcept>

#include <open3d/geometry/TriangleMesh.h>

#include "monotone_internal.hpp"

namespace crobot::patch_tree::detail {
Geometry geometry(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<int>& extra_bad,
    int normal_smoothing_iterations) {
  if (normal_smoothing_iterations < 0)
    throw std::invalid_argument(
        "Analysis normal smoothing iterations must be nonnegative");
  Geometry g;
  g.faces      = mesh.triangles_;
  g.vertices   = mesh.vertices_;
  const int nv = g.vertices.size(), nf = g.faces.size();
  if (!nf)
    return g;
  if (!nv)
    throw std::invalid_argument(
        "Monotone decomposition requires mesh vertices");
  Vec lo = g.vertices[0], hi = lo;
  for (const auto& p : g.vertices) {
    if (!p.allFinite())
      throw std::invalid_argument("Nonfinite analysis vertex");
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  g.diameter = (hi - lo).norm();
  if (!(g.diameter > 0) || !std::isfinite(g.diameter))
    throw std::invalid_argument("Invalid analysis mesh diameter");
  for (auto& p : g.vertices)
    p = (p - (lo + hi) / 2) / g.diameter;
  std::map<std::pair<int, int>, Edge> edges;
  std::vector<std::vector<int>> incident(nv);
  g.area.resize(nf);
  g.face_normals.resize(nf);
  g.centers.resize(nf);
  g.face_edges.resize(nf);
  g.bad.resize(nf, false);
  g.neighbors.resize(nf);
  std::vector<Vec> cross(nf);
  for (int i = 0; i < nf; i++) {
    const auto& f = g.faces[i];
    for (int j = 0; j < 3; j++) {
      if (f[j] < 0 || f[j] >= nv)
        throw std::invalid_argument("Analysis face index out of range");
      incident[f[j]].push_back(i);
      auto key = std::minmax(f[j], f[(j + 1) % 3]);
      auto& e  = edges[key];
      e.a      = key.first;
      e.b      = key.second;
      e.faces.push_back(i);
      e.starts.push_back(f[j]);
    }
    const Vec a = g.vertices[f[0]], b = g.vertices[f[1]], c = g.vertices[f[2]];
    cross[i]  = (b - a).cross(c - a);
    g.area[i] = cross[i].norm() / 2;
    if (!(g.area[i] > 0) || !std::isfinite(g.area[i]))
      throw std::invalid_argument(
          "Degenerate analysis face; prepare the mesh first");
    g.face_normals[i] = cross[i] / (2 * g.area[i]);
    g.centers[i]      = (a + b + c) / 3;
    g.total_area += g.area[i];
  }
  std::vector<bool> bad_vertex(nv, false);
  for (int v : extra_bad) {
    if (v < 0 || v >= nv)
      throw std::invalid_argument("Invalid nonmanifold vertex ID");
    bad_vertex[v] = true;
  }
  for (auto& [key, e] : edges) {
    e.safe   = e.faces.size() == 2 && e.starts[0] != e.starts[1];
    e.length = (g.vertices[e.a] - g.vertices[e.b]).norm();
    if (e.faces.size() > 2 || (e.faces.size() == 2 && !e.safe))
      bad_vertex[e.a] = bad_vertex[e.b] = true;
    if (e.safe) {
      g.neighbors[e.faces[0]].push_back(e.faces[1]);
      g.neighbors[e.faces[1]].push_back(e.faces[0]);
    }
    int id = g.edges.size();
    g.edges.push_back(e);
    for (int f : e.faces)
      for (int j = 0; j < 3; j++) {
        const std::pair<int, int> face_key
            = std::minmax(g.faces[f][j], g.faces[f][(j + 1) % 3]);
        if (face_key == key)
          g.face_edges[f][j] = id;
      }
  }
  // Detect disconnected vertex fans even when callers omit preparation
  // diagnostics.
  for (int v = 0; v < nv; v++)
    if (!bad_vertex[v] && incident[v].size() > 1) {
      std::set<int> remaining(incident[v].begin(), incident[v].end());
      std::vector<int> pending{*remaining.begin()};
      remaining.erase(pending[0]);
      while (!pending.empty()) {
        int f = pending.back();
        pending.pop_back();
        for (int j = 0; j < 3; j++) {
          const auto& e = g.edges[g.face_edges[f][j]];
          if (!e.safe || (e.a != v && e.b != v))
            continue;
          for (int q : e.faces)
            if (remaining.erase(q))
              pending.push_back(q);
        }
      }
      bad_vertex[v] = !remaining.empty();
    }
  for (int i = 0; i < nf; i++)
    for (int j = 0; j < 3; j++)
      g.bad[i] = g.bad[i] || bad_vertex[g.faces[i][j]];
  g.normals.assign(nv, Vec::Zero());
  std::vector<double> mass(nv, 0);
  for (int j = 0; j < 3; j++)
    for (int i = 0; i < nf; i++)
      if (!g.bad[i]) {
        g.normals[g.faces[i][j]] += cross[i];
        mass[g.faces[i][j]] += g.area[i] / 3;
      }
  for (auto& n : g.normals)
    n /= std::max(n.norm(), 1e-30);
  std::vector<std::vector<int>> smooth(nv);
  for (const auto& e : g.edges)
    if (e.safe && !bad_vertex[e.a] && !bad_vertex[e.b]
        && g.normals[e.a].dot(g.normals[e.b]) > std::cos(std::acos(-1.) / 6)) {
      smooth[e.a].push_back(e.b);
      smooth[e.b].push_back(e.a);
    }
  for (auto& row : smooth)
    std::sort(row.begin(), row.end());
  for (int pass = 0; pass < normal_smoothing_iterations; pass++) {
    auto next = g.normals;
    for (int v = 0; v < nv; v++) {
      Vec sum       = Vec::Zero();
      double weight = 0;
      for (int q : smooth[v])
        weight += mass[q];
      for (int q : smooth[v])
        sum += (mass[q] / std::max(weight, 1e-30)) * g.normals[q];
      if (weight > 0)
        next[v] = .5 * g.normals[v] + .5 * sum;
      next[v] /= std::max(next[v].norm(), 1e-30);
    }
    g.normals.swap(next);
  }
  g.mu.resize(nf);
  g.field.resize(nf);
  for (int i = 0; i < nf; i++) {
    const auto& f = g.faces[i];
    const Vec u = g.normals[f[0]], v = g.normals[f[1]], w = g.normals[f[2]],
              average = (u + v + w) / 3;
    g.field[i]        = average / std::max(average.norm(), 1e-30);
    g.mu[i][0]        = .5 * average.dot(cross[i]);
    g.mu[i][1]        = .5
                 * average.dot(
                     (u - w).cross(g.vertices[f[1]] - g.vertices[f[0]])
                     + (v - u).cross(g.vertices[f[2]] - g.vertices[f[0]]));
    g.mu[i][2] = .5 * u.dot(v.cross(w));
    g.bad[i]   = g.bad[i] || g.mu[i][0] <= 1e-20;
  }
  g.component.assign(nf, -1);
  for (int i = 0; i < nf; i++)
    if (g.component[i] < 0) {
      std::vector<int> pending{i};
      g.component[i] = g.components;
      while (!pending.empty()) {
        int f = pending.back();
        pending.pop_back();
        for (int q : g.neighbors[f])
          if (g.component[q] < 0) {
            g.component[q] = g.components;
            pending.push_back(q);
          }
      }
      g.components++;
    }
  return g;
}
} // namespace crobot::patch_tree::detail
