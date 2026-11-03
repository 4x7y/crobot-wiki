#pragma once

#include "crobot/math/math.hpp"

namespace crobot::planning {

/**
 * @brief Generate piecewise polynomials through multiple waypoints using
 * trapezoidal velocity profiles.
 *
 * This function computes a trajectory through a given set of input
 * WAYPOINTS. The function outputs positions, Q, velocities, QD, and
 * accelerations, QDD, at the given time samples, T. It also outputs the a
 * cell array of pp-forms of the polynomial for the trajectory in PP.
 *
 * @param way_points An N x P matrix specifying P waypoints that are each an
 * N x 1 vector of positions.
 * @param num_samples A scalar indicating the number of samples, M.
 * @param q The output vector trajectory position.
 * @param qd The output vector trajectory velocity.
 * @param qdd The output vector trajectory acceleration.
 * @param t The output time vector.
 */
void trapveltraj(
    const std::vector<math::VectorXd>& way_points,
    std::size_t num_samples,
    std::vector<math::VectorXd>& q,
    std::vector<math::VectorXd>& qd,
    std::vector<math::VectorXd>& qdd,
    std::vector<double>& t);

/**
 * @brief Compute parameters for a trapezoidal velocity profile.
 *
 * This function takes the user provided inputs and a set of waypoints and
 * outputs the corresponding completed set of parameters required for a
 * trapezoidal velocity profile: velocity, acceleration, acceleration time, and
 * final time. While all parameters are provided to this function, they will be
 * empty unless explicitly set by the user. If the user inputs cannot be
 * satisfied, then an error is output that indicates the constraint violation.
 *
 * @param i The index of the axis that the parameters correspond to.
 * @param j The index of the profile segment that the parameters correspond to.
 * Additionally, J and J+1 specify the bounding indices of WAYPOINTS of the
 * profile segment for which the parameters are being computed.
 * @param wayPoints
 * @param Vel The peak velocity of the profile segment.
 * @param Acc The acceleration of the profile segment.
 * @param TFi The duration of each trajectory segment.
 * @param TAc The duration of each of the acceleration phases of the segment.
 * @return The function outputs a complete set of the four scalar parameters
 * that may be used to generate a trapezoidal velocity profile for the profile
 * segment.
 */
std::tuple<double, double, double, double> computeProfileParams(
    std::size_t i,
    std::size_t j,
    const std::vector<math::VectorXd>& wayPoints,
    std::optional<std::vector<math::VectorXd>> Vel,
    std::optional<std::vector<math::VectorXd>> Acc,
    std::optional<std::vector<math::VectorXd>> TFi,
    std::optional<std::vector<math::VectorXd>> TAc);

/**
 * @brief Compute linear segment with polynomial blends.
 *
 * Compute scalar LSPB profile given velocity, acceleration, acceleration time,
 * final time, and a time vector for the profile segment between two waypoints.
 * This consists of three polynomial sections that together form a piecewise
 * polynomial.
 */
std::pair<math::Matrix3d, math::Vector4d> computeScalarLSPBCoefficients(
    double s0, double sF, double v, double a, double ta, double tf);

} // namespace crobot::planning

