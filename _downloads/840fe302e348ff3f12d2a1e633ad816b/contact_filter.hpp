#pragma once
#include <vector>

#include "crobot/geometry/mesh_patches.hpp"

namespace crobot::grasping {
// Historical three-finger span filter. Thresholds and units are preserved;
// this is a grasp-search heuristic, not a property of the mesh.
bool are_patch_centers_nearby(
    const geometry::MeshPatches& mesh, const std::vector<int>& patch_labels);
} // namespace crobot::grasping
