#include "crobot/patch_tree/patch_tree_io.hpp"

#include <iomanip>
#include <limits>
#include <locale>
#include <ostream>
#include <stdexcept>

#include <open3d/geometry/TriangleMesh.h>
#include <yaml-cpp/yaml.h>

namespace crobot::patch_tree {
namespace {

template <class Range>
void array(std::ostream& out, const Range& values) {
  out << '[';
  bool first = true;
  for (const auto& value : values) {
    if (!first)
      out << ',';
    first = false;
    out << +value; // uint8_t masks must be numbers, not characters.
  }
  out << ']';
}

template <class Range>
void rows(std::ostream& out, const Range& values) {
  if (values.empty())
    out << " []\n";
  else {
    out << '\n';
    for (const auto& value : values) {
      out << "    - ";
      array(out, value);
      out << '\n';
    }
  }
}

const char* status(PatchTree::BoundStatus value) {
  switch (value) {
    case PatchTree::BoundStatus::NotEvaluated:
      return "not_evaluated";
    case PatchTree::BoundStatus::Satisfied:
      return "satisfied";
    case PatchTree::BoundStatus::Indivisible:
      return "indivisible";
    case PatchTree::BoundStatus::RefinementLimit:
      return "refinement_limit";
  }
  throw std::invalid_argument("Invalid bound status");
}

const char* stage(PatchTree::Stage value) {
  switch (value) {
    case PatchTree::Stage::Unspecified:
      return "unspecified";
    case PatchTree::Stage::Object:
      return "object";
    case PatchTree::Stage::Component:
      return "component";
    case PatchTree::Stage::Monotone:
      return "monotone";
    case PatchTree::Stage::DetailGrouping:
      return "detail_grouping";
    case PatchTree::Stage::CurvatureRegion:
      return "curvature_region";
    case PatchTree::Stage::Spatial:
      return "spatial";
    case PatchTree::Stage::Normal:
      return "normal";
    case PatchTree::Stage::Torque:
      return "torque";
  }
  throw std::invalid_argument("Invalid generation stage");
}

YAML::Node options(const GeneratedPatchTree& result) {
  YAML::Node node;
  const auto& generation  = result.options;
  const auto& preparation = result.preparation_options;
  const auto& monotone    = generation.monotone;
#define FIELD(group, name) node[#group][#name] = group.name
  FIELD(generation, max_total_curvature);
  FIELD(generation, max_torque_variation);
  FIELD(generation, max_normal_angle);
  FIELD(generation, max_depth);
  FIELD(generation, split_by_curvature);
  FIELD(generation, max_coarse_faces);
  FIELD(generation, max_refinement_rounds);
  FIELD(generation, max_prepared_faces);
  FIELD(preparation, weld_exact_vertices);
  FIELD(preparation, orient_triangles);
  FIELD(preparation, orient_nested_shells);
  FIELD(preparation, normal_smoothing_iterations);
  FIELD(preparation, normal_smoothing_angle_degrees);
  FIELD(preparation, min_triangle_area);
  FIELD(monotone, normal_smoothing_iterations);
  for (double radius : monotone.radii)
    node["monotone"]["radii"].push_back(radius);
  FIELD(monotone, preferred_radius);
  FIELD(monotone, max_curvature_error);
  FIELD(monotone, rms_curvature_error);
  FIELD(monotone, max_uncertain_fraction);
  FIELD(monotone, tiny_area_fraction);
  FIELD(monotone, relief_limit);
  FIELD(monotone, diameter_limit);
  FIELD(monotone, normal_angle_degrees);
  FIELD(monotone, raw_crease_degrees);
  FIELD(monotone, parent_detail_fraction);
  FIELD(monotone, global_detail_fraction);
  FIELD(monotone, persistent_fraction);
  FIELD(monotone, persistent_limit_factor);
  FIELD(monotone, minimum_recipient_ratio);
  FIELD(monotone, ambiguous_boundary_share);
#undef FIELD
  return node;
}

} // namespace

void write_patch_tree_yaml(
    std::ostream& out, const GeneratedPatchTree& result) {
  if (!result.tree || !result.surface.mesh || !result.monotone)
    throw std::invalid_argument(
        "Cannot serialize an incomplete generated tree");
  const auto& tree = *result.tree;
  const auto& mesh = *result.surface.mesh;
  const auto& mono = *result.monotone;
  if (result.face_leaf_ids.size() != mesh.triangles_.size()
      || result.face_analysis_ids.size() != mesh.triangles_.size()
      || result.surface.face_source_ids.size() != mesh.triangles_.size()
      || mesh.vertex_normals_.size() != mesh.vertices_.size())
    throw std::invalid_argument("Generated mesh correspondence is incomplete");

  // Locale and precision must not change geometry or turn decimals into commas.
  const auto previous_locale    = out.getloc();
  const auto previous_precision = out.precision();
  const auto previous_flags     = out.flags();
  struct Restore
  {
    std::ostream& stream;
    std::locale locale;
    std::streamsize precision;
    std::ios::fmtflags flags;
    ~Restore() {
      stream.imbue(locale);
      stream.precision(precision);
      stream.flags(flags);
    }
  } restore{out, previous_locale, previous_precision, previous_flags};
  out.imbue(std::locale::classic());
  out << std::dec << std::defaultfloat << std::noshowpos
      << std::setprecision(std::numeric_limits<double>::max_digits10);
  YAML::Node header;
  header["format"]                   = "crobot.patch_tree";
  header["schema_version"]           = 2;
  header["root_id"]                  = 0;
  header["adjacency_policy"]         = "oriented_two_face_edges";
  header["bound_domain"]             = "prepared triangle vertices";
  header["refinement_limit_reached"] = result.refinement_limit_reached;
  header["refinement_stop_reason"]   = result.refinement_stop_reason;
  header["options"]                  = options(result);
  out << header << "\norigin: ";
  array(out, result.origin);
  out << "\n# Mesh coordinates and normals are the exact prepared bound "
         "domain.\nmesh:\n  vertices:";
  rows(out, mesh.vertices_);
  out << "  vertex_normals:";
  rows(out, mesh.vertex_normals_);
  out << "  triangles:";
  rows(out, mesh.triangles_);
  out << "  # One terminal node ID per prepared triangle; ancestors inherit "
         "support.\n  face_leaf_ids: ";
  array(out, result.face_leaf_ids);
  out << "\n  face_analysis_ids: ";
  array(out, result.face_analysis_ids);
  out << "\n  # One list of original input triangle IDs per prepared "
         "triangle.\n  face_source_ids:";
  rows(out, result.surface.face_source_ids);
  out << "  original_face_disposition: [";
  for (size_t i = 0; i < result.surface.original_face_disposition.size(); ++i) {
    if (i)
      out << ',';
    const auto d = result.surface.original_face_disposition[i];
    out
        << (d == geometry::OriginalFaceDisposition::Retained
                ? "retained"
                : d == geometry::OriginalFaceDisposition::Duplicate
                      ? "duplicate"
                      : "degenerate");
  }
  out << "]\nmonotone:\n  estimator: "
      << (mono.estimator == CurvatureEstimator::NormalField
              ? "normal_field"
              : "corrected_measures")
      << "\n  radius: " << mono.radius << "\n  diameter: " << mono.diameter
      << "\n  core_field_limits_met: "
      << (mono.core_field_limits_met ? "true" : "false")
      << "\n  # Fine, detail-grouped, uncertainty-absorbed region counts.\n  "
         "region_counts: ["
      << mono.cuts[0].regions.size() << ',' << mono.cuts[1].regions.size()
      << ',' << mono.cuts[2].regions.size() << "]\n  analysis_uncertain: ";
  array(out, mono.uncertain);
  out << "\n  analysis_absorbed_detail: ";
  array(out, mono.absorbed_detail);
  out << "\n  analysis_assigned_uncertain: ";
  array(out, mono.assigned_uncertain);
  out << "\n  analysis_ambiguous_assignment: ";
  array(out, mono.ambiguous_assignment);
  out << "\n# Bounds/status order: total curvature, normal angle (radians), "
         "torque span.\n"
         "# Curvature class: -1 mixed, 0 developable, 1 convex, 2 concave, 3 "
         "saddle, 4 uncertain.\n"
         "# descriptor_vertices bound normals/torques; they are not surface "
         "coordinates.\nnodes:\n";
  for (int id = 0; id < tree.get_num_nodes(); ++id) {
    const auto& n = tree.get_node_data(id);
    out << "  - {id: " << id << ", parent_id: " << n.parent_id
        << ", children_ids: ";
    array(out, tree.get_child_ids(id));
    out << ", type: " << n.type << ", stage: " << stage(n.stage)
        << ", curvature_class: " << n.curvature_class
        << ", monotone_region_id: " << n.monotone_region_id
        << ", uncertain_area_fraction: " << n.uncertain_area_fraction
        << ", face_count: " << n.face_ids.size() << ", bounds: ["
        << n.total_curvature << ',' << n.max_normal_angle << ','
        << n.torque_span << "], bound_status: [" << status(n.curvature_status)
        << ',' << status(n.normal_status) << ',' << status(n.torque_status)
        << "], descriptor_vertices: [";
    for (size_t v = 0; v < n.vertices.size(); ++v) {
      if (v)
        out << ',';
      array(out, n.vertices[v]);
    }
    out << "]}\n";
  }
  out << "# Every cut covers the whole prepared surface, including carried "
         "terminal nodes.\n"
         "# Edges are undirected pairs of node IDs; isolated patches remain in "
         "patch_ids.\n"
         "# parent_ids and child_ids align with patch_ids and link retained "
         "cuts.\n"
         "# tree_depth records the semantic depth; repeated partitions are "
         "omitted.\nlevels:\n";
  for (int level = 0; level < tree.get_num_levels(); ++level) {
    const auto& graph = tree.get_level_graph(level);
    out << "  - index: " << level
        << "\n    tree_depth: " << tree.get_level_depth(level)
        << "\n    patch_ids: [";
    bool first = true;
    for (const auto& [id, patch] : graph) {
      (void)patch;
      if (!first)
        out << ',';
      first = false;
      out << id;
    }
    out << "]\n    parent_ids: [";
    first = true;
    for (const auto& [id, patch] : graph) {
      (void)id;
      if (!first)
        out << ',';
      first = false;
      out << patch.parent_id;
    }
    out << "]\n    child_ids: [";
    first = true;
    for (const auto& [id, patch] : graph) {
      (void)id;
      if (!first)
        out << ',';
      first = false;
      array(out, patch.child_ids);
    }
    out << "]\n    edges: [";
    first = true;
    for (const auto& [id, patch] : graph)
      for (int neighbor : patch.neighbor_ids)
        if (id < neighbor) {
          if (!first)
            out << ',';
          first = false;
          out << '[' << id << ',' << neighbor << ']';
        }
    out << "]\n";
  }
  if (!out)
    throw std::runtime_error("Could not write patch tree YAML");
}

} // namespace crobot::patch_tree
