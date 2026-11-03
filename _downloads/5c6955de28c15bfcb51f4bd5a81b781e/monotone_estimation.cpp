// In-memory version of the validated normal-field estimator (round 3).
#include <algorithm>
#include <limits>
#include <queue>

#include "monotone_internal.hpp"
namespace crobot::patch_tree::detail {
namespace {
using V3 = Eigen::Vector3d;
using V5 = Eigen::Matrix<double, 5, 1>;
using M5 = Eigen::Matrix<double, 5, 5>;
struct Face
{
  V3 p, n;
  double area;
};
using Adjacency = std::vector<std::vector<std::pair<int, double>>>;
Adjacency adjacency(const Geometry& g, bool normal_field) {
  Adjacency adj(g.faces.size());
  for (const auto& e : g.edges)
    if (e.safe) {
      int a = e.faces[0], b = e.faces[1];
      const auto& normals = normal_field ? g.field : g.face_normals;
      if (g.bad[a] || g.bad[b]
          || normals[a].dot(normals[b]) < std::cos(65 * std::acos(-1.) / 180))
        continue;
      double d = (g.centers[a] - g.centers[b]).norm();
      adj[a].push_back({b, d});
      adj[b].push_back({a, d});
    }
  return adj;
}
std::vector<double> normal_estimate(const Geometry& g, double radius) {
  const size_t nf = g.faces.size();
  std::vector<Face> faces(nf);
  for (size_t i = 0; i < nf; i++)
    faces[i] = {g.centers[i], g.field[i], g.bad[i] ? 0. : g.area[i]};
  const auto adj = adjacency(g, true);
  std::vector<double> output(nf * 12, 0);
#pragma omp parallel
  {
    std::vector<double> distance(nf, std::numeric_limits<double>::infinity());
    std::vector<int> touched, hood;
    std::vector<double> weights, robust, residual, u, v;
    std::vector<V5> first, second;
#pragma omp for schedule(dynamic, 32)
    for (size_t index = 0; index < nf; index++) {
      const auto& origin = faces[index];
      if (origin.area <= 0 || origin.n.squaredNorm() < .5)
        continue;
      using Item = std::pair<double, int>;
      std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;
      distance[index] = 0;
      touched.push_back(index);
      heap.push({0, int(index)});
      hood.clear();
      bool truncated = false;
      double far     = 0;
      while (!heap.empty()) {
        auto item = heap.top();
        heap.pop();
        int i = item.second;
        if (item.first != distance[i])
          continue;
        if (hood.size() == 4000) {
          truncated = true;
          break;
        }
        if (faces[i].area > 0 && faces[i].n.dot(origin.n) > .2) {
          hood.push_back(i);
          far = std::max(far, item.first);
        }
        for (auto edge : adj[i]) {
          double d = item.first + edge.second;
          if (d > radius || d >= distance[edge.first])
            continue;
          if (!std::isfinite(distance[edge.first]))
            touched.push_back(edge.first);
          distance[edge.first] = d;
          heap.push({d, edge.first});
        }
      }
      double* out = output.data() + index * 12;
      out[7]      = hood.size();
      out[9]      = far;
      out[10]     = truncated;
      if (hood.size() >= 6 && !truncated) {
        V3 t = origin.n.unitOrthogonal(), b = origin.n.cross(t);
        size_t n    = hood.size();
        double mass = 0;
        weights.resize(n);
        robust.assign(n, 1.);
        residual.resize(n);
        first.resize(n);
        second.resize(n);
        u.resize(n);
        v.resize(n);
        for (size_t j = 0; j < n; j++) {
          int i    = hood[j];
          V3 d     = (faces[i].p - origin.p) / radius;
          double x = d.dot(t), y = d.dot(b);
          first[j] << 1, 0, x, y, 0;
          second[j] << 0, 1, 0, x, y;
          u[j]       = -faces[i].n.dot(t);
          v[j]       = -faces[i].n.dot(b);
          weights[j] = faces[i].area
                       * std::exp(-2 * std::pow(distance[i] / radius, 2));
          mass += weights[j];
        }
        V5 coefficients  = V5::Zero();
        double condition = 0;
        bool valid       = true;
        for (int it = 0; it < 5; it++) {
          M5 A = M5::Zero();
          V5 B = V5::Zero();
          for (size_t j = 0; j < n; j++) {
            double w = weights[j] * robust[j] / mass;
            A.noalias() += w
                           * (first[j] * first[j].transpose()
                              + second[j] * second[j].transpose());
            B.noalias() += w * (first[j] * u[j] + second[j] * v[j]);
          }
          Eigen::SelfAdjointEigenSolver<M5> eig(A, Eigen::EigenvaluesOnly);
          condition
              = eig.eigenvalues()[4] / std::max(1e-30, eig.eigenvalues()[0]);
          if (eig.eigenvalues()[0] <= 0 || condition > 1e8) {
            valid = false;
            break;
          }
          coefficients = A.ldlt().solve(B);
          for (size_t j = 0; j < n; j++)
            residual[j] = std::hypot(
                first[j].dot(coefficients) - u[j],
                second[j].dot(coefficients) - v[j]);
          auto sorted = residual;
          std::nth_element(
              sorted.begin(), sorted.begin() + n / 2, sorted.end());
          double scale = std::max(1e-5, 1.4826 * sorted[n / 2]);
          for (size_t j = 0; j < n; j++)
            robust[j]
                = std::min(1., 1.345 * scale / std::max(1e-30, residual[j]));
        }
        if (valid) {
          Eigen::Matrix2d S;
          S << coefficients[2], coefficients[3], coefficients[3],
              coefficients[4];
          S /= radius;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(S);
          double k0 = eig.eigenvalues()[0], k1 = eig.eigenvalues()[1], rss = 0,
                 mx = 0;
          for (size_t j = 0; j < n; j++) {
            rss += weights[j] * residual[j] * residual[j];
            mx = std::max(mx, residual[j]);
          }
          double rms = std::sqrt(rss / mass);
          out[0]     = k0 * k1;
          out[1]     = (k0 + k1) / 2;
          out[2]     = k0;
          out[3]     = k1;
          // Height residual is inapplicable (column 4 stays zero). Columns 5
          // and 6 report max and RMS tangent-normal residuals, expressed as
          // asin degrees.
          out[5]  = std::asin(std::min(1., mx)) * 180 / std::acos(-1.);
          out[6]  = std::asin(std::min(1., rms)) * 180 / std::acos(-1.);
          out[8]  = condition;
          out[11] = coefficients.allFinite() && out[6] <= 10.;
        }
      }
      for (int i : touched)
        distance[i] = std::numeric_limits<double>::infinity();
      touched.clear();
    }
  }
  return output;
}
std::vector<double> corrected_estimate(const Geometry& g, double radius) {
  const size_t nf = g.faces.size();
  const auto adj  = adjacency(g, false);
  std::vector<double> output(nf * 12, 0);
#pragma omp parallel
  {
    std::vector<double> distance(nf, std::numeric_limits<double>::infinity());
    std::vector<int> touched, hood;
#pragma omp for schedule(dynamic, 32)
    for (size_t index = 0; index < nf; index++) {
      using Item = std::pair<double, int>;
      std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;
      distance[index] = 0;
      touched.push_back(index);
      heap.push({0, int(index)});
      hood.clear();
      bool truncated = false;
      double far     = 0;
      while (!heap.empty()) {
        auto item = heap.top();
        heap.pop();
        int i = item.second;
        if (item.first != distance[i])
          continue;
        if (hood.size() == 4000) {
          truncated = true;
          break;
        }
        hood.push_back(i);
        far = std::max(far, item.first);
        for (auto edge : adj[i]) {
          double d = item.first + edge.second;
          if (d > radius || d >= distance[edge.first])
            continue;
          if (!std::isfinite(distance[edge.first]))
            touched.push_back(edge.first);
          distance[edge.first] = d;
          heap.push({d, edge.first});
        }
      }
      Vec sum = Vec::Zero();
      for (int i : hood)
        if (!g.bad[i])
          sum += std::exp(-2 * std::pow(distance[i] / radius, 2)) * g.mu[i];
      double* out = output.data() + index * 12;
      out[7]      = hood.size();
      out[9]      = far;
      out[10]     = truncated;
      if (sum[0] > 1e-20 && !truncated) {
        double K = sum[2] / sum[0], H = -sum[1] / (2 * sum[0]),
               disc = H * H - K;
        out[0]      = K;
        out[1]      = H;
        out[2]      = H - std::sqrt(std::max(0., disc));
        out[3]      = H + std::sqrt(std::max(0., disc));
        out[8]      = std::max(0., -disc) / std::max(1., std::max(K, H * H));
        out[11]     = std::isfinite(K) && std::isfinite(H) && out[8] < .25;
      }
      for (int i : touched)
        distance[i] = std::numeric_limits<double>::infinity();
      touched.clear();
    }
  }
  return output;
}
} // namespace
Evidence estimate(
    const Geometry& g,
    double radius,
    CurvatureEstimator method,
    const MonotoneOptions& options) {
  auto raw = method == CurvatureEstimator::NormalField
                 ? normal_estimate(g, radius)
                 : corrected_estimate(g, radius);
  Evidence result;
  size_t nf = g.faces.size();
  result.values.resize(nf);
  result.preferred.resize(nf);
  result.costs.resize(nf);
  result.invalid.resize(nf);
  const double tolerance
      = std::min(options.max_curvature_error, options.rms_curvature_error);
  for (size_t i = 0; i < nf; i++) {
    std::copy_n(raw.data() + 12 * i, 12, result.values[i].begin());
    auto& v   = result.values[i];
    double k0 = v[2], k1 = v[3];
    result.invalid[i]   = v[11] == 0 || g.bad[i];
    result.preferred[i] = 0;
    if (k1 < -tolerance)
      result.preferred[i] = 1;
    if (k0 > tolerance)
      result.preferred[i] = 2;
    if (k0 < -tolerance && k1 > tolerance)
      result.preferred[i] = 3;
    result.costs[i]
        = {{std::min(std::abs(k0), std::abs(k1)),
            std::max(k1, 0.),
            std::max(-k0, 0.),
            std::max({k0, -k1, 0.})}};
    if (result.invalid[i]) {
      result.preferred[i] = 4;
      result.costs[i].fill(0);
    }
  }
  return result;
}
} // namespace crobot::patch_tree::detail
