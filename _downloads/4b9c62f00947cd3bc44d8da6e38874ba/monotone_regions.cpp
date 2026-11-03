#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>

#include "monotone_internal.hpp"

namespace crobot::patch_tree::detail {
void update_cut(const Geometry& g, const Evidence& evidence, MonotoneCut& cut) {
  const int count = cut.regions.size();
  std::vector<double> mass(count, 0), unknown(count, 0), squared(count, 0);
  for (auto& r : cut.regions) {
    r.max_curvature_error = 0;
    r.rms_curvature_error = 0;
  }
  for (size_t f = 0; f < g.faces.size(); f++) {
    int id       = cut.face_regions[f];
    auto& region = cut.regions[id];
    int kind     = static_cast<int>(region.kind);
    mass[id] += g.area[f];
    if (evidence.invalid[f])
      unknown[id] += g.area[f];
    if (kind < 4 && !evidence.invalid[f]) {
      double error               = evidence.costs[f][kind];
      region.max_curvature_error = std::max(region.max_curvature_error, error);
      squared[id] += g.area[f] * error * error;
    }
  }
  for (int i = 0; i < count; i++) {
    auto& r              = cut.regions[i];
    r.area_fraction      = mass[i] / g.total_area;
    r.uncertain_fraction = unknown[i] / mass[i];
    r.rms_curvature_error
        = std::sqrt(squared[i] / std::max(mass[i] - unknown[i], 1e-30));
  }
  std::map<std::pair<int, int>, double> lengths;
  for (const auto& edge : g.edges)
    if (edge.safe) {
      int a = cut.face_regions[edge.faces[0]],
          b = cut.face_regions[edge.faces[1]];
      if (a != b)
        lengths[std::minmax(a, b)] += edge.length;
    }
  cut.graph_edges.clear();
  cut.graph_shared_length.clear();
  for (const auto& [key, length] : lengths) {
    cut.graph_edges.push_back({key.first, key.second});
    cut.graph_shared_length.push_back(length);
  }
}

MonotoneCut merge_regions(
    const Geometry& g,
    const Evidence& evidence,
    const MonotoneOptions& options) {
  const int nf = g.faces.size();
  std::vector<int> seed(nf, -1);
  int count = 0;
  for (int f = 0; f < nf; f++)
    if (seed[f] < 0) {
      std::vector<int> pending{f};
      seed[f] = count;
      while (!pending.empty()) {
        int i = pending.back();
        pending.pop_back();
        for (int q : g.neighbors[i])
          if (seed[q] < 0 && evidence.preferred[q] == evidence.preferred[i]) {
            seed[q] = count;
            pending.push_back(q);
          }
      }
      count++;
    }
  const int size = 2 * count;
  std::vector<int> parent(size), types(size, 4);
  std::iota(parent.begin(), parent.end(), 0);
  std::vector<bool> active(size, false);
  std::fill_n(active.begin(), count, true);
  std::vector<double> mass(size, 0), unknown(size, 0);
  std::vector<std::array<double, 4>> sums(size), maxima(size);
  for (int f = 0; f < nf; f++) {
    int r = seed[f];
    mass[r] += g.area[f];
    unknown[r] += g.area[f] * evidence.invalid[f];
    types[r] = evidence.preferred[f];
    for (int k = 0; k < 4; k++) {
      sums[r][k] += g.area[f] * evidence.costs[f][k] * evidence.costs[f][k];
      maxima[r][k] = std::max(maxima[r][k], evidence.costs[f][k]);
    }
  }
  std::vector<std::map<int, double>> adjacency(size);
  for (const auto& edge : g.edges)
    if (edge.safe) {
      int a = seed[edge.faces[0]], b = seed[edge.faces[1]];
      if (a != b) {
        adjacency[a][b] += edge.length;
        adjacency[b][a] += edge.length;
      }
    }
  using Candidate = std::tuple<double, int, int, int>;
  std::
      priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>>
          heap;
  auto push = [&](int a, int b) {
    double total = mass[a] + mass[b], bad = unknown[a] + unknown[b],
           known = total - bad;
    if (known < 1e-15) {
      heap.emplace(0., std::min(a, b), std::max(a, b), 4);
      return;
    }
    if (bad / total > options.max_uncertain_fraction)
      return;
    int kind    = -1;
    double best = std::numeric_limits<double>::infinity(), mx = 0;
    for (int k = 0; k < 4; k++) {
      double m = std::max(maxima[a][k], maxima[b][k]),
             r = std::sqrt((sums[a][k] + sums[b][k]) / known);
      if (m <= options.max_curvature_error + 1e-12
          && r <= options.rms_curvature_error + 1e-12 && r < best) {
        best = r;
        kind = k;
        mx   = m;
      }
    }
    if (kind < 0)
      return;
    heap.emplace(
        best + .02 * mx + 1e-9 * total / std::max(adjacency[a][b], 1e-20),
        std::min(a, b),
        std::max(a, b),
        kind);
  };
  for (int a = 0; a < count; a++)
    for (auto [b, length] : adjacency[a])
      if (a < b)
        push(a, b);
  int next = count;
  while (!heap.empty()) {
    auto [score, a, b, kind] = heap.top();
    heap.pop();
    (void)score;
    if (!active[a] || !active[b])
      continue;
    int c     = next++;
    parent[a] = parent[b] = c;
    active[a] = active[b] = false;
    active[c]             = true;
    mass[c]               = mass[a] + mass[b];
    unknown[c]            = unknown[a] + unknown[b];
    types[c]              = kind;
    for (int k = 0; k < 4; k++) {
      sums[c][k]   = sums[a][k] + sums[b][k];
      maxima[c][k] = std::max(maxima[a][k], maxima[b][k]);
    }
    std::set<int> neighbors;
    for (auto [q, l] : adjacency[a])
      neighbors.insert(q);
    for (auto [q, l] : adjacency[b])
      neighbors.insert(q);
    neighbors.erase(a);
    neighbors.erase(b);
    for (int q : neighbors) {
      double length = adjacency[q][a] + adjacency[q][b];
      adjacency[q].erase(a);
      adjacency[q].erase(b);
      adjacency[q][c] = length;
      adjacency[c][q] = length;
    }
    adjacency[a].clear();
    adjacency[b].clear();
    for (auto [q, l] : adjacency[c])
      push(c, q);
  }
  std::vector<int> roots;
  for (int i = 0; i < next; i++)
    if (active[i])
      roots.push_back(i);
  std::stable_sort(roots.begin(), roots.end(), [&](int a, int b) {
    return mass[a] > mass[b];
  });
  std::vector<int> lookup(next, -1);
  for (size_t i = 0; i < roots.size(); i++)
    lookup[roots[i]] = i;
  for (int i = next - 1; i >= 0; i--)
    parent[i] = parent[parent[i]];
  MonotoneCut cut;
  cut.face_regions.resize(nf);
  cut.regions.resize(roots.size());
  for (int f = 0; f < nf; f++)
    cut.face_regions[f] = lookup[parent[seed[f]]];
  for (size_t i = 0; i < roots.size(); i++)
    cut.regions[i].kind = static_cast<CurvatureClass>(types[roots[i]]);
  cut.fine_to_region.resize(roots.size());
  std::iota(cut.fine_to_region.begin(), cut.fine_to_region.end(), 0);
  cut.previous_to_region = cut.fine_to_region;
  cut.active_nodes       = cut.fine_to_region;
  update_cut(g, evidence, cut);
  return cut;
}
} // namespace crobot::patch_tree::detail
