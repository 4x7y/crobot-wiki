#pragma once
#include <map>
#include <set>

#include <Eigen/Dense>

#include "crobot/patch_tree/monotone_decomposition.hpp"

namespace crobot::patch_tree::detail {
using Vec    = Eigen::Vector3d;
using Values = std::array<double, 12>;
struct Edge
{
  int a = 0, b = 0;
  std::vector<int> faces, starts;
  bool safe     = false;
  double length = 0;
};
struct Geometry
{
  double diameter = 0, total_area = 0;
  int components = 0;
  std::vector<Vec> vertices, normals, face_normals, field, centers, mu;
  std::vector<Eigen::Vector3i> faces, face_edges;
  std::vector<Edge> edges;
  std::vector<double> area;
  std::vector<bool> bad;
  std::vector<int> component;
  std::vector<std::vector<int>> neighbors;
};
struct Evidence
{
  std::vector<Values> values;
  std::vector<int> preferred;
  std::vector<std::array<double, 4>> costs;
  std::vector<bool> invalid;
};
Geometry geometry(
    const open3d::geometry::TriangleMesh&,
    const std::vector<int>&,
    int normal_smoothing_iterations = 2);
Evidence estimate(
    const Geometry&, double, CurvatureEstimator, const MonotoneOptions&);
MonotoneCut merge_regions(
    const Geometry&, const Evidence&, const MonotoneOptions&);
void update_cut(const Geometry&, const Evidence&, MonotoneCut&);
MonotoneCut group_details(
    const Geometry&,
    const Evidence&,
    const MonotoneCut&,
    const std::vector<const Evidence*>&,
    const MonotoneOptions&,
    std::vector<std::uint8_t>&);
MonotoneCut assign_uncertain(
    const Geometry&,
    const Evidence&,
    const MonotoneCut&,
    const MonotoneOptions&,
    MonotoneDecomposition&);
inline double angle(const Vec& a, const Vec& b) {
  return std::acos(std::max(-1., std::min(1., a.dot(b)))) * 180
         / std::acos(-1.);
}
} // namespace crobot::patch_tree::detail
