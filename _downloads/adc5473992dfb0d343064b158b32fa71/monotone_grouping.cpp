#include <algorithm>
#include <limits>
#include <numeric>
#include <tuple>

#include "monotone_internal.hpp"

namespace crobot::patch_tree::detail {
namespace {
struct Interface
{
  double length = 0, weighted_angle = 0, raw_angle = 0, field_angle = 0;
  bool bad = false;
};
std::map<int, Interface> interfaces(
    const Geometry& g, const std::vector<int>& labels, int donor) {
  std::map<int, Interface> result;
  for (const auto& edge : g.edges)
    if (edge.safe) {
      int fa = edge.faces[0], fb = edge.faces[1], a = labels[fa],
          b = labels[fb];
      if ((a == donor) == (b == donor))
        continue;
      auto& v    = result[a == donor ? b : a];
      double raw = angle(g.face_normals[fa], g.face_normals[fb]);
      v.length += edge.length;
      v.weighted_angle += edge.length * raw;
      v.raw_angle   = std::max(v.raw_angle, raw);
      v.field_angle = std::max(v.field_angle, angle(g.field[fa], g.field[fb]));
      v.bad         = v.bad || g.bad[fa] || g.bad[fb];
    }
  return result;
}

MonotoneCut finish(
    const Geometry& g,
    const Evidence& evidence,
    const MonotoneCut& base,
    const std::vector<int>& owner,
    const std::vector<int>& nodes,
    const std::vector<std::array<int, 4>>& history,
    const std::vector<std::uint8_t>& detail,
    const std::vector<std::uint8_t>& assigned,
    const std::vector<std::uint8_t>& ambiguous,
    const std::vector<double>& prominence,
    const std::vector<double>& normal_error) {
  const int n = owner.size();
  std::vector<double> mass(n, 0);
  std::vector<int> roots;
  for (size_t f = 0; f < g.faces.size(); f++)
    mass[owner[base.face_regions[f]]] += g.area[f];
  for (int i = 0; i < n; i++)
    if (owner[i] == i)
      roots.push_back(i);
  std::stable_sort(roots.begin(), roots.end(), [&](int a, int b) {
    return mass[a] > mass[b];
  });
  std::vector<int> lookup(n, -1);
  for (size_t i = 0; i < roots.size(); i++)
    lookup[roots[i]] = i;
  MonotoneCut cut;
  cut.previous_to_region.resize(n);
  for (int i = 0; i < n; i++)
    cut.previous_to_region[i] = lookup[owner[i]];
  for (int i : base.fine_to_region)
    cut.fine_to_region.push_back(cut.previous_to_region[i]);
  cut.merge_tree = history;
  cut.regions.resize(roots.size());
  for (int root : roots) {
    cut.active_nodes.push_back(nodes[root]);
    cut.regions[lookup[root]].kind = base.regions[root].kind;
  }
  for (size_t f = 0; f < g.faces.size(); f++) {
    int root = owner[base.face_regions[f]], r = lookup[root];
    cut.face_regions.push_back(r);
    auto& data = cut.regions[r];
    data.detail_fraction += g.area[f] * detail[f] / mass[root];
    data.assigned_fraction += g.area[f] * assigned[f] / mass[root];
    data.ambiguous_fraction += g.area[f] * ambiguous[f] / mass[root];
  }
  for (int i = 0; i < n; i++) {
    auto& r             = cut.regions[cut.previous_to_region[i]];
    r.max_detail_relief = std::max(r.max_detail_relief, prominence[i]);
    r.max_detail_normal_angle
        = std::max(r.max_detail_normal_angle, normal_error[i]);
  }
  update_cut(g, evidence, cut);
  return cut;
}

double relief(
    const Geometry& g,
    const std::vector<int>& faces,
    const MonotoneOptions& options) {
  const double failure = std::numeric_limits<double>::infinity();
  std::set<int> vertices;
  std::map<int, int> edges;
  for (int f : faces) {
    if (g.bad[f])
      return failure;
    for (int j = 0; j < 3; j++) {
      vertices.insert(g.faces[f][j]);
      edges[g.face_edges[f][j]]++;
    }
  }
  Vec lo = g.vertices[*vertices.begin()], hi = lo;
  for (int v : vertices) {
    lo = lo.cwiseMin(g.vertices[v]);
    hi = hi.cwiseMax(g.vertices[v]);
  }
  if ((hi - lo).norm() > options.diameter_limit)
    return failure;
  std::map<int, double> weights;
  int boundary_count = 0;
  double total       = 0;
  for (auto [id, count] : edges)
    if (count == 1) {
      const auto& e = g.edges[id];
      weights[e.a] += e.length / 2;
      weights[e.b] += e.length / 2;
      total += e.length;
      boundary_count++;
    }
  if (boundary_count < 3 || total <= 0)
    return failure;
  Vec origin = Vec::Zero();
  for (auto [id, w] : weights)
    origin += (w / total) * g.vertices[id];
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (auto [id, w] : weights) {
    Vec d = g.vertices[id] - origin;
    covariance += (w / total) * d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  if (eig.info() != Eigen::Success
      || eig.eigenvalues()[1] < std::max(1e-20, eig.eigenvalues()[2] * 1e-8))
    return failure;
  double maximum = 0;
  for (int id : vertices)
    maximum = std::max(
        maximum,
        std::abs((g.vertices[id] - origin).dot(eig.eigenvectors().col(0))));
  return maximum;
}
} // namespace

MonotoneCut group_details(
    const Geometry& g,
    const Evidence& evidence,
    const MonotoneCut& base,
    const std::vector<const Evidence*>& scales,
    const MonotoneOptions& options,
    std::vector<std::uint8_t>& absorbed) {
  const int n = base.regions.size(), nf = g.faces.size();
  std::vector<int> owner(n);
  std::iota(owner.begin(), owner.end(), 0);
  auto labels = base.face_regions, nodes = base.active_nodes;
  auto history = base.merge_tree;
  std::vector<double> mass(n, 0), unknown(n, 0), exception(n, 0),
      prominence(n, 0), normal_error(n, 0), initial_unknown(n, 0);
  std::vector<std::vector<int>> members(n);
  std::vector<int> donors;
  for (int f = 0; f < nf; f++) {
    int id = labels[f];
    members[id].push_back(f);
    mass[id] += g.area[f];
    unknown[id] += g.area[f] * evidence.invalid[f];
  }
  for (int i = 0; i < n; i++) {
    initial_unknown[i] = unknown[i] / mass[i];
    if (mass[i] < options.tiny_area_fraction * g.total_area)
      donors.push_back(i);
  }
  std::stable_sort(donors.begin(), donors.end(), [&](int a, int b) {
    return mass[a] < mass[b];
  });
  absorbed.assign(nf, 0);
  std::vector<std::uint8_t> none(nf, 0);
  double global = 0;
  std::vector<double> measured(n, -1), effective(n, options.relief_limit);
  for (int donor : donors) {
    measured[donor]   = relief(g, members[donor], options);
    double persistent = 0;
    int kind          = static_cast<int>(base.regions[donor].kind);
    if (kind < 4)
      for (int f : members[donor]) {
        bool same = true;
        for (const auto* s : scales)
          same = same && s->preferred[f] == kind;
        if (same)
          persistent += g.area[f];
      }
    if (persistent / mass[donor] >= options.persistent_fraction)
      effective[donor] *= options.persistent_limit_factor;
  }
  bool changed = true;
  while (changed) {
    changed = false;
    for (int donor : donors) {
      if (owner[donor] != donor || measured[donor] > effective[donor] + 1e-12)
        continue;
      auto adjacent = interfaces(g, labels, donor);
      std::tuple<double, double, int> best{
          std::numeric_limits<double>::infinity(), 0, -1};
      double selected_angle = 0;
      for (auto [recipient, edge] : adjacent) {
        double union_mass = mass[recipient] + mass[donor];
        if (mass[recipient] < std::max(
                options.tiny_area_fraction * g.total_area,
                options.minimum_recipient_ratio * mass[donor]))
          continue;
        if ((global + mass[donor]) / g.total_area
            > options.global_detail_fraction + 1e-12)
          continue;
        if ((exception[recipient] + mass[donor]) / union_mass
            > options.parent_detail_fraction + 1e-12)
          continue;
        if ((unknown[recipient] + unknown[donor]) / union_mass
            > std::max(
                  options.max_uncertain_fraction, initial_unknown[recipient])
                  + 1e-12)
          continue;
        if (edge.bad || edge.raw_angle > options.raw_crease_degrees
            || edge.field_angle > options.normal_angle_degrees)
          continue;
        auto score = std::make_tuple(
            measured[donor] / effective[donor]
                + .1 * edge.field_angle / options.normal_angle_degrees,
            -edge.length,
            recipient);
        if (score < best) {
          best           = score;
          selected_angle = edge.field_angle;
        }
      }
      int recipient = std::get<2>(best);
      if (recipient < 0)
        continue;
      int parent = base.fine_to_region.size() + history.size();
      history.push_back(
          {parent,
           nodes[recipient],
           nodes[donor],
           static_cast<int>(base.regions[recipient].kind)});
      nodes[recipient] = parent;
      owner[donor]     = recipient;
      for (int f : members[donor]) {
        labels[f]   = recipient;
        absorbed[f] = 1;
      }
      mass[recipient] += mass[donor];
      unknown[recipient] += unknown[donor];
      exception[recipient] += mass[donor];
      global += mass[donor];
      prominence[recipient] = std::max(prominence[recipient], measured[donor]);
      normal_error[recipient]
          = std::max(normal_error[recipient], selected_angle);
      changed = true;
    }
  }
  return finish(
      g,
      evidence,
      base,
      owner,
      nodes,
      history,
      absorbed,
      none,
      none,
      prominence,
      normal_error);
}

MonotoneCut assign_uncertain(
    const Geometry& g,
    const Evidence& evidence,
    const MonotoneCut& base,
    const MonotoneOptions& options,
    MonotoneDecomposition& result) {
  const int n = base.regions.size(), nf = g.faces.size();
  std::vector<int> owner(n);
  std::iota(owner.begin(), owner.end(), 0);
  auto labels = base.face_regions, nodes = base.active_nodes;
  auto history = base.merge_tree;
  std::vector<double> mass(n, 0), prominence(n), normal_error(n);
  for (int f = 0; f < nf; f++)
    mass[labels[f]] += g.area[f];
  for (int i = 0; i < n; i++) {
    prominence[i]   = base.regions[i].max_detail_relief;
    normal_error[i] = base.regions[i].max_detail_normal_angle;
  }
  result.assigned_uncertain.assign(nf, 0);
  result.ambiguous_assignment.assign(nf, 0);
  while (true) {
    std::tuple<double, double, int> best{1, 0, -1};
    UncertainAssignment chosen;
    for (int donor = 0; donor < n; donor++)
      if (owner[donor] == donor
          && base.regions[donor].kind == CurvatureClass::Uncertain) {
        auto adjacent = interfaces(g, labels, donor);
        UncertainAssignment candidate;
        candidate.donor = donor;
        double total    = 0;
        for (auto [recipient, e] : adjacent)
          if (base.regions[recipient].kind != CurvatureClass::Uncertain) {
            candidate.candidates.push_back(
                {recipient, e.length, e.weighted_angle / e.length});
            total += e.length;
          }
        if (candidate.candidates.empty())
          continue;
        std::sort(
            candidate.candidates.begin(),
            candidate.candidates.end(),
            [&](const auto& a, const auto& b) {
              return std::make_tuple(
                         -a.shared_length,
                         a.mean_interface_angle,
                         -mass[a.recipient],
                         a.recipient)
                     < std::make_tuple(
                         -b.shared_length,
                         b.mean_interface_angle,
                         -mass[b.recipient],
                         b.recipient);
            });
        candidate.recipient = candidate.candidates[0].recipient;
        candidate.boundary_share
            = candidate.candidates[0].shared_length / total;
        candidate.area_fraction = mass[donor] / g.total_area;
        candidate.ambiguous
            = candidate.boundary_share < options.ambiguous_boundary_share;
        auto score = std::make_tuple(
            -candidate.boundary_share,
            -candidate.candidates[0].shared_length,
            donor);
        if (score < best) {
          best   = score;
          chosen = candidate;
        }
      }
    if (chosen.donor < 0)
      break;
    int donor = chosen.donor, recipient = chosen.recipient,
        parent = base.fine_to_region.size() + history.size();
    history.push_back(
        {parent,
         nodes[recipient],
         nodes[donor],
         static_cast<int>(base.regions[recipient].kind)});
    nodes[recipient] = parent;
    owner[donor]     = recipient;
    for (int f = 0; f < nf; f++)
      if (labels[f] == donor) {
        labels[f]                      = recipient;
        result.assigned_uncertain[f]   = 1;
        result.ambiguous_assignment[f] = chosen.ambiguous;
      }
    result.assignments.push_back(std::move(chosen));
  }
  return finish(
      g,
      evidence,
      base,
      owner,
      nodes,
      history,
      result.absorbed_detail,
      result.assigned_uncertain,
      result.ambiguous_assignment,
      prominence,
      normal_error);
}
} // namespace crobot::patch_tree::detail
