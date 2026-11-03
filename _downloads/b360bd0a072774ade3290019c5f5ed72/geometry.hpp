#pragma once

#include <memory>

// Load before hpp-fcl to setup custom Eigen types
#include "crobot/math/matrix.hpp"

// hpp-fcl includes appear below matrix.hpp to include custom Eigen types
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace crobot::collision {

//==============================================================================
// Direct type aliases to hpp-fcl collision geometry classes
//==============================================================================

using CollisionGeometry = hpp::fcl::CollisionGeometry;
using CollisionObject   = hpp::fcl::CollisionObject;
using GeometryPtr       = std::shared_ptr<CollisionGeometry>;
using ConstGeometryPtr  = std::shared_ptr<const CollisionGeometry>;
using Transform3d       = hpp::fcl::Transform3f;

using Sphere    = hpp::fcl::Sphere;
using Box       = hpp::fcl::Box;
using Capsule   = hpp::fcl::Capsule;
using Cylinder  = hpp::fcl::Cylinder;
using Ellipsoid = hpp::fcl::Ellipsoid;
using Plane     = hpp::fcl::Plane;
using Mesh      = hpp::fcl::BVHModel<hpp::fcl::OBBRSS>;

//==============================================================================
// Helper functions for converting between crobot and hpp-fcl types
//==============================================================================

/**
 * @brief Convert crobot Matrix4d transform to hpp-fcl Transform3f.
 */
Transform3d to_fcl_transform(const math::Matrix4d& m);

/**
 * @brief Convert hpp-fcl Transform3f to crobot Matrix4d.
 */
math::Matrix4d from_fcl_transform(const Transform3d& tf);

/**
 * @brief Create a mesh collision geometry from vertices and triangles.
 *
 * @param vertices Matrix of vertices (N x 3)
 * @param triangles Matrix of triangle indices (M x 3)
 * @return std::shared_ptr<Mesh> The created mesh geometry
 */
std::shared_ptr<Mesh> create_mesh(
    const math::Points& vertices, const math::Triangles& triangles);

} // namespace crobot::collision
