#include "crobot/collision/collision_body.hpp"

using namespace crobot::collision;

//==============================================================================
void CollisionBody::add_collision_geometry(
    const std::string& name, GeometryPtr geom, const math::Matrix4d& tform) {
  std::string key = name;
  if (key.empty()) {
    // MuJoCo/URDF imports can contain unnamed geoms. Keep them addressable by
    // assigning a unique internal key instead of failing on duplicate "".
    key = "__unnamed_geom_" + std::to_string(m_geoms.size());
    while (m_name2geom.find(key) != m_name2geom.end()) {
      key = "__unnamed_geom_" + std::to_string(m_name2geom.size());
    }
  }

  if (m_name2geom.find(key) != m_name2geom.end()) {
    throw std::runtime_error("Geometry with name '" + key + "' already exists");
  }
  m_geoms.push_back(geom);
  m_name2geom[key]  = geom;
  m_name2tform[key] = tform;
}

//==============================================================================
void CollisionBody::add_geom_sphere(
    const std::string& name, double radius, const math::Matrix4d& tform) {
  auto geom = std::make_shared<Sphere>(radius);
  add_collision_geometry(name, geom, tform);
}

//==============================================================================
void CollisionBody::add_geom_capsule(
    const std::string& name,
    double radius,
    double length,
    const math::Matrix4d& tform) {
  auto geom = std::make_shared<Capsule>(radius, length);
  add_collision_geometry(name, geom, tform);
}

//==============================================================================
void CollisionBody::add_geom_ellipsoid(
    const std::string& name,
    double ra,
    double rb,
    double rc,
    const math::Matrix4d& tform) {
  auto geom = std::make_shared<Ellipsoid>(ra, rb, rc);
  add_collision_geometry(name, geom, tform);
}

//==============================================================================
void CollisionBody::add_geom_cylinder(
    const std::string& name,
    double radius,
    double height,
    const math::Matrix4d& tform) {
  auto geom = std::make_shared<Cylinder>(radius, height);
  add_collision_geometry(name, geom, tform);
}

//==============================================================================
void CollisionBody::add_geom_box(
    const std::string& name,
    double lx,
    double ly,
    double lz,
    const math::Matrix4d& tform) {
  auto geom = std::make_shared<Box>(lx, ly, lz);
  add_collision_geometry(name, geom, tform);
}

//==============================================================================
void CollisionBody::add_geom_mesh(
    const std::string& name,
    const math::Points& vertices,
    const math::Triangles& triangles,
    const math::Matrix4d& tform) {
  auto geom = create_mesh(vertices, triangles);
  add_collision_geometry(name, geom, tform);
}

//==============================================================================
std::pair<GeometryPtr, crobot::math::Matrix4d> CollisionBody::get_geom(
    const std::string& name) const {
  return {m_name2geom.at(name), m_name2tform.at(name)};
}
