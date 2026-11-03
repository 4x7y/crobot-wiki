#include "crobot/collision/geometry.hpp"

namespace crobot::collision {

//==============================================================================
// Helper functions
//==============================================================================

Transform3d to_fcl_transform(const math::Matrix4d& m) {
  hpp::fcl::Matrix3f R;
  hpp::fcl::Vec3f t;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R(i, j) = static_cast<float>(m(i, j));
    }
    t[i] = static_cast<float>(m(i, 3));
  }

  return Transform3d(R, t);
}

math::Matrix4d from_fcl_transform(const Transform3d& tf) {
  math::Matrix4d m = math::Matrix4d::Identity();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      m(i, j) = static_cast<double>(tf.getRotation()(i, j));
    }
    m(i, 3) = static_cast<double>(tf.getTranslation()[i]);
  }

  return m;
}

std::shared_ptr<Mesh> create_mesh(
    const math::Points& vertices, const math::Triangles& triangles) {
  auto mesh = std::make_shared<Mesh>();

  mesh->beginModel();

  // Add vertices
  std::vector<hpp::fcl::Vec3f> verts;
  verts.reserve(vertices.rows());
  for (int i = 0; i < vertices.rows(); ++i) {
    hpp::fcl::Vec3f v(
        static_cast<float>(vertices(i, 0)),
        static_cast<float>(vertices(i, 1)),
        static_cast<float>(vertices(i, 2)));
    verts.push_back(v);
  }

  // Add triangles
  std::vector<hpp::fcl::Triangle> tris;
  tris.reserve(triangles.rows());
  for (int i = 0; i < triangles.rows(); ++i) {
    hpp::fcl::Triangle t(
        static_cast<size_t>(triangles(i, 0)),
        static_cast<size_t>(triangles(i, 1)),
        static_cast<size_t>(triangles(i, 2)));
    tris.push_back(t);
  }

  mesh->addSubModel(verts, tris);
  mesh->endModel();

  return mesh;
}

} // namespace crobot::collision
