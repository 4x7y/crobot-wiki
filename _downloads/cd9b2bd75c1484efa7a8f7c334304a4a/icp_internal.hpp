#pragma once

#include <functional>

#include "crobot/geometry/icp.hpp"

namespace crobot::geometry::detail {

void validate_points(const ICPPoints& points);
void validate_transform(const math::Matrix4d& transform);
void validate_options(const ICPOptions& options);
ICPPoints transform_points(
    const ICPPoints& points, const math::Matrix4d& transform);

using Correspondences
    = std::function<ICPStatus(const math::Matrix4d&, ICPPoints&, ICPPoints&)>;

// A successful correspondence query returns Converged as its success code.
ICPResult run_icp(
    const math::Matrix4d& initial_transform,
    const ICPOptions& options,
    const Correspondences& correspondences);

} // namespace crobot::geometry::detail
