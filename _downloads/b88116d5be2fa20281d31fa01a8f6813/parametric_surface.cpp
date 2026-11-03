#include "crobot/math/parametric_surface.hpp"

using namespace crobot;
using namespace math;

// //==============================================================================
// Matrix32 ParametricSurface::normal_jacobian(const math::Vector2d& uv) {
//   return normal_numerical_jacobian(uv);
// }

// //==============================================================================
// Matrix32 ParametricSurface::normal_numerical_jacobian(
//     const math::Vector2d& uv) {
//   Matrix32 jac;
//   double epsilon = 1e-6;
//   jac.col(0) = (normal(uv + Vector2d(epsilon, 0.0))
//                 - normal(uv + Vector2d(-epsilon, 0.0)))
//                / (2. * epsilon);
//   jac.col(1) = (normal(uv + Vector2d(0.0, epsilon))
//                 - normal(uv + Vector2d(0.0, -epsilon)))
//                / (2. * epsilon);

//   return jac;
// }
