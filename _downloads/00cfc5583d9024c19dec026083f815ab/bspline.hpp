#pragma once

#include <vector>

#include "crobot/math/array.hpp"
#include "crobot/math/common.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::math {

/**
 * @brief Find the span of the given parameter in the knot vector.
 *
 * @param[in] degree Degree of the curve.
 * @param[in] knots Knot vector of the curve.
 * @param[in] u Parameter value.
 * @return Span index into the knot vector such that (span - 1) < u <= span
 */
int find_span(int degree, const std::vector<double>& knots, double u);

/**
 * @brief Compute a single B-spline basis function
 *
 * @param[in] i The ith basis function to compute.
 * @param[in] deg Degree of the basis function.
 * @param[in] knots Knot vector corresponding to the basis functions.
 * @param[in] u Parameter to evaluate the basis functions at.
 * @return The value of the ith basis function at u.
 */
double bspline_one_basis(
    int i, int deg, const std::vector<double>& U, double u);

/**
 * @brief Compute all non-zero B-spline basis functions
 *
 * @param[in] deg Degree of the basis function.
 * @param[in] span Index obtained from find_span() corresponding the u and
 * knots.
 * @param[in] knots Knot vector corresponding to the basis functions.
 * @param[in] u Parameter to evaluate the basis functions at.
 * @return N Values of (deg+1) non-zero basis functions.
 */
std::vector<double> bspline_basis(
    int deg, int span, const std::vector<double>& knots, double u);

/**
 * @brief Compute all non-zero derivatives of B-spline basis functions
 *
 * @param[in] deg Degree of the basis function.
 * @param[in] span Index obtained from find_span() corresponding the u and
 * knots.
 * @param[in] knots Knot vector corresponding to the basis functions.
 * @param[in] u Parameter to evaluate the basis functions at.
 * @param[in] num_ders Number of derivatives to compute (num_ders <= deg)
 * @return ders Values of non-zero derivatives of basis functions.
 */
array2<double> bspline_der_basis(
    int deg,
    int span,
    const std::vector<double>& knots,
    double u,
    int num_ders);

} // namespace crobot::math

