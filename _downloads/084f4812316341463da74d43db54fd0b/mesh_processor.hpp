#pragma once

/**
 * @file mesh_processor.hpp
 * @brief Main header for mesh processing and decomposition functionality
 *
 * This module provides tools for decomposing 3D meshes into patches based on
 * curvature and torque characteristics. It's useful for grasp planning and
 * contact region analysis.
 *
 * Main classes:
 * - MeshUtils: Utility functions for mesh operations
 * - MeshPatch: Represents a surface patch on a mesh
 * - MeshPatchTree: Deterministic generator for connected surface patches
 * - MeshDecomposer: Mesh preparation, refinement, and planning-tree creation
 *
 * Saved-data planning uses crobot::geometry::PatchMeshData (surface and
 * adjacency), crobot::geometry::MeshPatches (contact vertices/centers), and
 * crobot::patch_tree::PatchTree (the decomposition hierarchy and its level
 * graphs). Live generation returns the same crobot::patch_tree::PatchTree,
 * together with the prepared surface, original-face correspondence, origin, and
 * options in GeneratedPatchTree. Keep that result for later planning, and do
 * not mutate its surface: face IDs, normal bounds, torque bounds, and adjacency
 * all refer to that specific prepared mesh and inherited normal field.
 *
 * Bounds certify the sampled vertex normals and vertex torques. They do not
 * certify continuous contact interpolation, collision-free hand motion, or
 * force closure. Torque units follow the mesh coordinate units and the
 * supplied origin; load meshes in the same coordinate frame as the planner.
 *
 * Example usage:
 * @code
 * #include <crobot/patch_tree/mesh_processor.hpp>
 *
 * // Load the surface and construct a reusable planning hierarchy.
 * auto mesh = crobot::geometry::MeshIO::load_mesh("object.stl");
 * crobot::patch_tree::PatchGenerationOptions options;
 * options.max_torque_variation = 0.02; // mesh length units, unit normal force
 * auto generated = crobot::patch_tree::MeshDecomposer::generate_patch_tree(
 *     mesh, mesh->GetCenter(), options);
 * const auto& tree = *generated.tree;
 *
 * // Every level covers all prepared faces. Same-level edges support coarse
 * // motion planning; child links refine a patch in the following level.
 * // Repeated partitions are omitted. Graph parent/child links can skip
 * // semantic tree nodes; get_level_depth(level) gives the original depth.
 * for (int level = 0; level < tree.get_num_levels(); ++level) {
 *   for (const auto& [node_id, patch] : tree.get_level_graph(level)) {
 *     const auto& neighbors = patch.neighbor_ids;
 *     const auto& children = patch.child_ids;
 *     // A terminal patch carried to a deeper level links to its own ID.
 *     // The final level has no children. Isolated patches have no neighbors.
 *   }
 * }
 *
 * // Resource-limited or indivisible leaves retain their surface support and
 * // explicit status; a planner must not treat them as satisfying all bounds.
 * std::vector<int> unresolved;
 * for (int id : tree.get_leaves(0)) {
 *   const auto& data = tree.get_node_data(id);
 *   using Status = crobot::patch_tree::PatchTree::BoundStatus;
 *   if (data.curvature_status != Status::Satisfied ||
 *       data.normal_status != Status::Satisfied ||
 *       (options.max_torque_variation > 0 &&
 *        data.torque_status != Status::Satisfied)) {
 *     unresolved.push_back(id);
 *   }
 *   // Exact surface support indexes generated.surface.mesh->triangles_.
 *   const auto& prepared_faces = data.face_ids;
 *   const auto& original_faces = data.source_face_ids;
 *   // Optional compact export preserves the inherited normals:
 *   auto surface = crobot::geometry::MeshUtils::extract_triangles(
 *       *generated.surface.mesh, prepared_faces, true);
 * }
 * // generated.refinement_stop_reason explains a refinement stop.
 * @endcode
 *
 * PatchNode::get_vertices() returns normal/torque bounding-polytope points
 * used by legacy grasp tests, not surface vertices. Use per-node face_ids
 * for contact geometry and get_num_levels() for the compact graph level count;
 * get_height()/get_l1()/get_l2() retain their saved-tree estimates.
 */

#include "crobot/geometry/gaussian_curvature.hpp"
#include "crobot/geometry/mesh_io.hpp"
#include "crobot/geometry/mesh_utils.hpp"
#include "crobot/patch_tree/mesh_decomposer.hpp"
#include "crobot/patch_tree/patch_tree_io.hpp"
#include "crobot/patch_tree/mesh_patch.hpp"
#include "crobot/patch_tree/mesh_patch_tree.hpp"
