#pragma once

#include <string>

#include "crobot/math/matrix.hpp"

namespace crobot::gui {

// Perspective projection matrix
// xscale/yscale: scaling on x/y axes
// z_near/z_far: clip distances
math::Matrix4f persp(float xscale, float yscale, float z_near, float z_far);

// Returns 'look at' matrix
// IMPORTANT: fw is normalized forward direction,
// NOT the point to look at!
// pos: camera position
// fw: forward direction (must be normalized)
// up: up direction for camera (must be normalized)
math::Matrix4f look_at(
    const Eigen::Ref<const math::Vector3f>& pos,
    const Eigen::Ref<const math::Vector3f>& fw,
    const Eigen::Ref<const math::Vector3f>& up);

// Estimate normals given points in pointcloud with given *triangles*
// outputs normals into out. If faces is empty, same as 'no element buffer'
// version. NOTE: out must already be of same size as verts
void estimate_normals(
    const Eigen::Ref<const math::Points>& verts,
    const Eigen::Ref<const math::Triangles>& faces,
    Eigen::Ref<math::Points> out);

// Estimate normals given points in pointcloud with no element buffer
// (point 0,1,2 are 1st triangle, 3,4,5 2nd, etc..)
// outputs normals into out.
// NOTE: out must already be of same size as verts
void estimate_normals(
    const Eigen::Ref<const math::Points>& verts, Eigen::Ref<math::Points> out);

// From list of triangles and uv triangles (same # rows),
// construct a map from uv vertex indices to vertex indices
Eigen::Matrix<math::Index, Eigen::Dynamic, 1> make_uv_to_vert_map(
    size_t num_uv_verts,
    const Eigen::Ref<const math::Triangles>& tri_faces,
    const Eigen::Ref<const math::Triangles>& uv_tri_faces);

} // namespace crobot::gui

