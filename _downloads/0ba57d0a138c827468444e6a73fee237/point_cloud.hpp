#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::gui {

// Represents a 3D point cloud with vertices (including uv, normals)
// where each vertex has a color.
// Also supports drawing the points as a polyline (call draw_lines()).
class PointCloud {
public:
  explicit PointCloud(size_t num_verts = 0);
  // Set vertices with positions pos with colors rgb
  explicit PointCloud(
      const Eigen::Ref<const math::Points>& pos,
      const Eigen::Ref<const math::Points>& rgb);
  // Set all points to same color
  // (can't put Eigen::Vector3f due 'ambiguity' with above)
  explicit PointCloud(
      const Eigen::Ref<const math::Points>& pos,
      float r = 1.f,
      float g = 1.f,
      float b = 1.f);

  ~PointCloud();

  // Resize point cloud (destroys current data)
  void resize(size_t num_verts);

  // Draw mesh to shader wrt camera
  void draw(math::Index shader_id, const Camera& camera);

  // Position part of verts
  inline Eigen::Ref<math::Points> verts_pos() {
    return data.leftCols<3>();
  }
  // RGB part of verts
  inline Eigen::Ref<math::Points> verts_rgb() {
    return data.rightCols<3>();
  }

  // Enable/disable object
  PointCloud& enable(bool val = true);
  // Set the point size for drawing
  inline PointCloud& set_point_size(float val) {
    point_size = val;
    return *this;
  }
  // Draw polylines between consecutive points
  inline PointCloud& draw_lines() {
    lines = true;
    return *this;
  }

  // Apply translation
  PointCloud& translate(const Eigen::Ref<const math::Vector3f>& vec);
  // Set translation
  PointCloud& set_translation(const Eigen::Ref<const math::Vector3f>& vec);

  // Apply rotation (3x3 rotation matrix)
  PointCloud& rotate(const Eigen::Ref<const math::Matrix3f>& mat);
  // Apply scaling
  PointCloud& scale(const Eigen::Ref<const math::Vector3f>& vec);
  // Apply uniform scaling in each dimension
  PointCloud& scale(float val);

  // Apply transform (4x4 affine transform matrix)
  PointCloud& apply_transform(const Eigen::Ref<const math::Matrix4f>& mat);
  // Set transform (4x4 affine transform matrix)
  PointCloud& set_transform(const Eigen::Ref<const math::Matrix4f>& mat);

  // Init or update VAO/VBO buffers from current vertex data
  // Must called before first draw for each GLFW context to ensure
  // textures are reconstructed.
  // force_init: INTERNAL, whether to force recreating buffers, DO NOT use
  // this
  void update(bool force_init = false);

  // ADVANCED: Free buffers. Used automatically in destructor.
  void free_bufs();

  // * Example point clouds/lines
  static PointCloud Line(
      const Eigen::Ref<const math::Vector3f>& a,
      const Eigen::Ref<const math::Vector3f>& b,
      const Eigen::Ref<const math::Vector3f>& color
      = math::Vector3f(1.f, 1.f, 1.f));

  // Data store
  math::PointsRGB data;

  // Whether this point cloud is enabled; if false, does not draw anything
  bool enabled = true;

  // If true, draws polylines between vertices
  // If false (default), draws points only
  bool lines = false;

  // Point size (if lines = false)
  float point_size = 1.f;

  // Model local transfom
  math::Matrix4f transform;

private:
  // Buffer indices
  math::Index VAO = -1, VBO = -1;
};

} // namespace crobot::gui

