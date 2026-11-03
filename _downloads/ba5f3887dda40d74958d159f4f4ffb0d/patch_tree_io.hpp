#pragma once

#include <iosfwd>

#include "crobot/patch_tree/mesh_decomposer.hpp"

namespace crobot::patch_tree {

// Write the versioned, self-contained YAML representation of a generated tree.
// IDs are zero based. Node support is recovered from mesh.face_leaf_ids by
// following node parents; face_source_ids maps prepared faces to input faces.
// Each compact level stores its original tree_depth, active patch IDs, edges,
// and parallel parent_ids/child_ids arrays linking adjacent retained cuts.
// Node children_ids are semantic tree links and may include omitted cuts.
// The final level has no outgoing child links. Descriptor vertices describe
// normal/torque bounds, not mesh positions. Original uncertainty is retained.
// Streams arrays incrementally to avoid a second in-memory copy of the tree.
// Throws on missing generated data or output failure. No legacy CSV is needed.
void write_patch_tree_yaml(
    std::ostream& output, const GeneratedPatchTree& result);

} // namespace crobot::patch_tree
