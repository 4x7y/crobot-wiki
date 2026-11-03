#pragma once

#include <memory>

#include "crobot/collision/geometry.hpp"

namespace crobot::collision {

// Forward declaration
struct CollisionInfo;

class CollisionBody : public std::enable_shared_from_this<CollisionBody> {
public:
  /**
   * @brief Destroy the Collision Body object
   */
  virtual ~CollisionBody() = default;

  /**
   * @brief Add a Collision CollisionGeometry instance to this body.
   *
   * @param geom CollisionGeometry to be added.
   */
  void add_collision_geometry(
      const std::string& name, GeometryPtr geom, const math::Matrix4d& tform);

  /**
   * @brief Add a sphere geometry.
   *
   * The sphere is centered at the geom’s position. Only one size parameter is
   * used, specifying the radius of the sphere.
   *
   * @param radius Radius of the sphere.
   * @param tform Pose of the geom frame, represented as a transformation matrix
   * in the body frame.
   */
  void add_geom_sphere(
      const std::string& name, double radius, const math::Matrix4d& tform);

  /**
   * @brief Add a capsule geometry.
   *
   * The capsule type defines a capsule, which is a cylinder capped with two
   * half-spheres. It is oriented along the Z axis of the geom’s frame. When the
   * geom frame is specified in the usual way, two size parameters are required:
   * the radius of the capsule followed by the height of the cylinder part.
   *
   * @param radius Radius of the capsule.
   * @param length Length of the cylinder part.
   * @param tform Pose of the geom frame, represented as a transformation matrix
   * in the body frame.
   */
  void add_geom_capsule(
      const std::string& name,
      double radius,
      double length,
      const math::Matrix4d& tform);

  /**
   * @brief Add a ellipsoid geometry.
   *
   * This is a sphere scaled separately along the X, Y and Z axes of the local
   * frame. It requires three size parameters, corresponding to the three radii.
   * Note that even though ellipsoids are smooth, their collisions are handled
   * via the general-purpose convex collider. The only exception are
   * plane-ellipsoid collisions which are computed analytically.
   *
   * @param ra Radius along the X axis.
   * @param rb Radius along the Y axis.
   * @param rc Radius along the Z axis.
   * @param tform Pose of the geom frame, represented as a transformation
   * matrix in the body frame.
   */
  void add_geom_ellipsoid(
      const std::string& name,
      double ra,
      double rb,
      double rc,
      const math::Matrix4d& tform);

  /**
   * @brief Add a cylinder geometry.
   *
   * A cylinder requires two size parameters: the radius and height of the
   * cylinder. The cylinder is oriented along the Z axis of the geom’s frame.
   *
   * @param radius Radius of the cylinder.
   * @param height Length of the cylinder.
   * @param tform Pose of the geom frame, represented as a transformation
   * matrix in the body frame.
   */
  void add_geom_cylinder(
      const std::string& name,
      double radius,
      double height,
      const math::Matrix4d& tform);

  /**
   * @brief Add a box geometry.
   *
   * Three size parameters are required, corresponding to the half-sizes of the
   * box along the X, Y and Z axes of the geom’s frame. Note that box-box
   * collisions are the only pair-wise collision type that can generate a large
   * number of contact points, up to 8 depending on the configuration.
   *
   * @param lx Size of the box along the X axis.
   * @param ly Size of the box along the Y axis.
   * @param lz Size of the box along the Z axis.
   * @param tform Pose of the geom frame, represented as a transformation
   * matrix in the body frame.
   */
  virtual void add_geom_box(
      const std::string& name,
      double lx,
      double ly,
      double lz,
      const math::Matrix4d& tform);

  /**
   * @brief Add a mesh geometry.
   *
   * @param vertices Coordinates of the mesh vertices in the geom frame.
   * @param triangles Faces of the mesh, represented as a tuple of three vertex
   * indices.
   * @param tform Pose of the geom frame, represented as a transformation matrix
   * in the body frame.
   */
  virtual void add_geom_mesh(
      const std::string& name,
      const math::Points& vertices,
      const math::Triangles& triangles,
      const math::Matrix4d& tform);

  /**
   * @brief Get a collision geometry and its transform by name.
   *
   * @param name Name of the geometry to retrieve.
   * @return A pair containing the geometry pointer and its transform.
   */
  std::pair<GeometryPtr, math::Matrix4d> get_geom(
      const std::string& name) const;

private:
  /// A set of collision geometries that belongs to this collision body.
  std::vector<GeometryPtr> m_geoms;

  std::unordered_map<std::string, GeometryPtr> m_name2geom;

  /// Transform of each geometry with respect to the body frame.
  std::unordered_map<std::string, math::Matrix4d> m_name2tform;
};

} // namespace crobot::collision
