#pragma once

#include <vector>

#include "crobot/geometry/mesh_preprocessor.hpp"

namespace crobot::geometry {

/**
 * Split all edges of marked faces and conformingly retriangulate neighbors.
 * Midpoint positions stay on the existing piecewise planar contact surface.
 * Normals interpolate the prepared field; patch normals are not recomputed.
 * Returns the old prepared-face ID of each new face and updates both original
 * face correspondence maps. Preserves components, UVs, colors and materials.
 * Invalid indices, antipodal normals, and exhausted coordinate precision throw
 * without changing prepared. Empty markings are a no-op. Resource and round
 * limits belong to the caller.
 */
std::vector<int> refine_marked_faces(
    PreparedMesh& prepared, const std::vector<int>& marked_faces);

} // namespace crobot::geometry
