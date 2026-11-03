#include "crobot/patch_tree/mesh_decomposer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <set>
#include <stdexcept>

#include <open3d/geometry/TriangleMesh.h>

#include "crobot/geometry/mesh_io.hpp"
#include "crobot/geometry/mesh_refinement.hpp"
#include "crobot/geometry/mesh_utils.hpp"

namespace crobot::patch_tree {
using geometry::MeshPreparationOptions;
using geometry::MeshUtils;
using geometry::prepare_mesh;
using geometry::refine_marked_faces;

GeneratedPatchTree MeshDecomposer::generate_patch_tree(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
    const Eigen::Vector3d& origin,
    const PatchGenerationOptions& options,
    const MeshPreparationOptions& preparation) {
  if (!mesh)
    throw std::invalid_argument("Cannot decompose a null mesh");
  GeneratedPatchTree result;
  result.origin              = origin;
  result.options             = options;
  result.preparation_options = preparation;
  result.surface             = prepare_mesh(*mesh, preparation);
  result.analysis_mesh       = result.surface.mesh;
  result.face_analysis_ids.resize(result.surface.mesh->triangles_.size());
  std::iota(
      result.face_analysis_ids.begin(), result.face_analysis_ids.end(), 0);
  if (options.split_by_curvature)
    result.monotone
        = std::make_shared<MonotoneDecomposition>(decompose_monotone(
            *result.analysis_mesh,
            options.monotone,
            result.surface.report.nonmanifold_vertex_ids));
  for (int round = 0;; ++round) {
    MeshPatchTree generated(
        result.surface.mesh,
        origin,
        options,
        result.monotone,
        result.face_analysis_ids);
    for (size_t f = 0; f < result.surface.mesh->triangles_.size(); ++f)
      generated.add_triangle(static_cast<int>(f));
    std::vector<int> marked;
    for (const auto& entry : generated.get_nodes()) {
      const auto& node = entry.second;
      if (!node.children_ids.empty() || node.data.face_ids.size() != 1)
        continue;
      if (node.data.curvature_status == PatchTree::BoundStatus::Indivisible
          || node.data.normal_status == PatchTree::BoundStatus::Indivisible
          || node.data.torque_status == PatchTree::BoundStatus::Indivisible)
        marked.push_back(node.data.face_ids.front());
    }
    if (!marked.empty()) {
      const size_t faces = result.surface.mesh->triangles_.size();
      std::set<std::pair<int, int>> split_edges;
      for (int id : marked) {
        const auto& t = result.surface.mesh->triangles_[id];
        for (int j = 0; j < 3; ++j)
          split_edges.insert(std::minmax(t[j], t[(j + 1) % 3]));
      }
      size_t refined_faces = faces;
      for (const auto& t : result.surface.mesh->triangles_)
        for (int j = 0; j < 3; ++j)
          refined_faces += split_edges.count(std::minmax(t[j], t[(j + 1) % 3]));
      if (round >= options.max_refinement_rounds) {
        result.refinement_limit_reached = true;
        result.refinement_stop_reason
            = "Maximum mesh refinement rounds reached";
      }
      else if (refined_faces > options.max_prepared_faces) {
        result.refinement_limit_reached = true;
        result.refinement_stop_reason   = "Prepared face budget reached";
      }
      else {
        try {
          const auto new_to_old = refine_marked_faces(result.surface, marked);
          std::vector<int> mapping;
          mapping.reserve(new_to_old.size());
          for (int face : new_to_old)
            mapping.push_back(result.face_analysis_ids.at(face));
          result.face_analysis_ids = std::move(mapping);
          continue;
        }
        catch (const std::domain_error& error) {
          result.refinement_stop_reason = error.what();
        }
      }
    }
    result.tree = generated.get_planning_tree(result.surface.face_source_ids);
    result.face_leaf_ids = generated.get_face_leaf_ids();
    for (int id : result.tree->get_leaves(0)) {
      const auto& data = result.tree->get_node_data(id);
      if (data.curvature_status == PatchTree::BoundStatus::RefinementLimit
          || data.normal_status == PatchTree::BoundStatus::RefinementLimit
          || data.torque_status == PatchTree::BoundStatus::RefinementLimit) {
        result.refinement_limit_reached = true;
        if (result.refinement_stop_reason.empty())
          result.refinement_stop_reason = "Maximum hierarchy depth reached";
      }
    }
    return result;
  }
}

void MeshDecomposer::decompose_mesh(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
    const Eigen::Vector3d& origin,
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>>&
        submeshes_torque,
    bool save_mesh,
    const std::string& base_directory,
    double max_torque_variation) {
  PatchGenerationOptions options;
  options.max_torque_variation = max_torque_variation;
  const auto generated         = generate_patch_tree(mesh, origin, options);
  if (save_mesh && !base_directory.empty())
    std::filesystem::create_directories(base_directory);
  // Build into a local collection so failure does not append partial output.
  std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> patches;
  for (int id : generated.tree->get_leaves(0)) {
    const auto& ids = generated.tree->get_face_ids(id);
    if (ids.empty())
      continue;
    auto patch
        = MeshUtils::extract_triangles(*generated.surface.mesh, ids, true);
    const double hue = std::fmod(id * 0.618033988749895, 1.0);
    patch->PaintUniformColor(
        Eigen::Vector3d(0.25 + 0.7 * hue, 0.3 + 0.5 * (1 - hue), 0.65));
    if (save_mesh) {
      const auto filename = std::filesystem::path(base_directory)
                            / ("patch_" + std::to_string(id) + ".ply");
      geometry::MeshIO::save_mesh(filename.string(), *patch);
    }
    patches.push_back(std::move(patch));
  }
  submeshes_torque.insert(
      submeshes_torque.end(), patches.begin(), patches.end());
}

std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>>
MeshDecomposer::generate_submeshes(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh) {
  if (!mesh)
    throw std::invalid_argument("Cannot prepare a null mesh");
  auto prepared       = prepare_mesh(*mesh);
  const auto monotone = decompose_monotone(
      *prepared.mesh, {}, prepared.report.nonmanifold_vertex_ids);
  std::vector<std::vector<int>> regions(monotone.cuts[2].regions.size());
  for (size_t f = 0; f < prepared.mesh->triangles_.size(); ++f)
    regions[monotone.cuts[2].face_regions[f]].push_back(f);
  std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> result;
  for (const auto& region : regions)
    result.push_back(
        MeshUtils::extract_triangles(*prepared.mesh, region, true));
  return result;
}

} // namespace crobot::patch_tree
