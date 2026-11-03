#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace open3d::geometry {
class TriangleMesh;
}

namespace crobot::patch_tree {

enum class CurvatureClass
{
  NearDevelopable = 0,
  Convex          = 1,
  Concave         = 2,
  Saddle          = 3,
  Uncertain       = 4
};
enum class CurvatureEstimator
{
  CorrectedMeasures = 0,
  NormalField       = 1
};

// Dimensionless lengths use the original analysis mesh bounding-box diagonal D.
// These are the accepted round-six settings; region count is never a target.
struct MonotoneOptions
{
  // Analysis-only normal filtering, independent of preparation/contact normals.
  // Zero keeps the area-weighted vertex normals; mesh vertices never move.
  int normal_smoothing_iterations = 2;
  std::array<double, 3> radii{{0.02, 0.04, 0.08}};
  double preferred_radius         = 0.04;
  double max_curvature_error      = 1.0;
  double rms_curvature_error      = 0.5;
  double max_uncertain_fraction   = 0.05;
  double tiny_area_fraction       = 0.001;
  double relief_limit             = 0.002;
  double diameter_limit           = 0.08;
  double normal_angle_degrees     = 20.0;
  double raw_crease_degrees       = 45.0;
  double parent_detail_fraction   = 0.05;
  double global_detail_fraction   = 0.02;
  double persistent_fraction      = 0.8;
  double persistent_limit_factor  = 0.25;
  double minimum_recipient_ratio  = 5.0;
  double ambiguous_boundary_share = 0.6;
};

struct MonotoneRegion
{
  CurvatureClass kind  = CurvatureClass::Uncertain;
  double area_fraction = 0, uncertain_fraction = 0;
  double max_curvature_error = 0, rms_curvature_error = 0;
  double max_detail_relief = 0, max_detail_normal_angle = 0;
  double detail_fraction = 0, assigned_fraction = 0, ambiguous_fraction = 0;
};

struct MonotoneCut
{
  std::vector<int> face_regions;
  std::vector<MonotoneRegion> regions;
  // Fine-to-coarse: identity for cut 0; otherwise maps the preceding cut.
  std::vector<int> previous_to_region;
  std::vector<int> fine_to_region;
  std::vector<std::array<int, 2>> graph_edges;
  std::vector<double> graph_shared_length;
  // Binary parents of cut-zero leaves. Rows: node, left, right, kind.
  std::vector<std::array<int, 4>> merge_tree;
  std::vector<int> active_nodes;
};

struct UncertainCandidate
{
  int recipient        = -1;
  double shared_length = 0, mean_interface_angle = 0;
};
struct UncertainAssignment
{
  int donor = -1, recipient = -1; // IDs in cut 1
  double area_fraction = 0, boundary_share = 0;
  bool ambiguous = false;
  std::vector<UncertainCandidate> candidates;
};

struct MonotoneDecomposition
{
  double diameter              = 0;
  CurvatureEstimator estimator = CurvatureEstimator::NormalField;
  double radius                = 0.04;
  bool core_field_limits_met   = false;
  int components               = 0;
  // Original fine regions -> small-detail grouping -> uncertainty absorption.
  std::array<MonotoneCut, 3> cuts;
  std::vector<double> gaussian_curvature, mean_curvature;
  std::vector<std::array<double, 2>> principal_curvatures;
  std::vector<std::uint8_t> uncertain, absorbed_detail, assigned_uncertain,
      ambiguous_assignment;
  std::vector<UncertainAssignment> assignments;
  // Filtered analysis normals are separate from contact/bound normals.
  std::vector<Eigen::Vector3d> analysis_normals;
};

// Input must have finite, nondegenerate triangles and consistent orientation.
// prepare_mesh() provides cleanup/provenance for arbitrary input meshes.
// No geometry or supplied normals are mutated; uncertain estimates stay
// flagged.
MonotoneDecomposition decompose_monotone(
    const open3d::geometry::TriangleMesh& mesh,
    const MonotoneOptions& options               = {},
    const std::vector<int>& nonmanifold_vertices = {});

} // namespace crobot::patch_tree
