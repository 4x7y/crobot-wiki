#pragma once

// clang-format off
#include <vector>
#include "crobot/common/assert.hpp"
#include "crobot/math/matrix.hpp"
#include <unsupported/Eigen/Polynomials>
// clang-format on

namespace crobot::math {

// Polynomials are equations of a single variable with nonnegative integer
// exponents. In this implementation, we represents polynomials with numeric
// vectors containing the polynomial coefficients ordered by descending power.
// For example, [1 -4 4] corresponds to x^2 - 4x + 4.

/**
 * @brief Evaluate polynomial.
 *
 * Y = POLYVAL(P,X) returns the value of a polynomial P evaluated at X. P is a
 * vector of length N+1 whose elements are the coefficients of the polynomial in
 * descending powers:
 *
 *           Y = P(0)*X^N + P(1)*X^(N-1) + ... + P(N-1)*X + P(N)
 */
double polyval(const Eigen::Ref<const VectorXd> p, double x);

/**
 * @brief Evaluate a 2D polynomial at point (x, y).
 *
 * F = POLYVAL2D(C,X,Y) returns the value of a 2D polynomial C evaluated at (X,
 * Y). The coefficient matrix C is a (N+1) by (M+1) scalar matrix, where N and M
 * are the highest order of x and y in the polynomial, respectively. The
 * coefficients are defined as follows:
 *
 * P(X,Y) = C(0,0)*X^N*Y^M     + C(0,1)*X^N*Y^(M-1)     + ... + C(0,M)*X^N     +
 *          C(1,0)*X^(N-1)*Y^M + C(1,1)*X^(N-1)*Y^(M-1) + ... + C(1,M)*X^(N-1) +
 *          ...
 *          C(N,0)*Y^M         + C(N,1)*Y^(M-1)         + ... + C(N,M)
 *
 * @param c Array of coefficients ordered so that the coefficient of the term of
 * multi-degree i, j is contained in c(N-i, M-j).
 *
 * For example, the 2D polynomial
 *
 *               p(x, y) = 2x^2*y + 3y - 6
 *
 *                       = 2 x^2*y  +  0 x^2  +
 *                         0 x  *y  +  0 x    +
 *                         3     y  + -6
 *
 * is represented as a coefficent matrix
 *
 *                     c = [ 2,  0]
 *                         [ 0,  0]
 *                         [ 3, -6].
 */
double polyval2d(const Eigen::Ref<const MatrixXd> c, double x, double y);

/**
 * @brief Add one polynomial to another.
 *
 * POLYADD(U,V) returns the sum of two polynomials U+V. The arguments are
 * sequence of coefficients from highest order term to lowest, i.e., [1,2,3]
 * represents the polynomial X^2+2X+3.
 */
VectorXd polyadd(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v);

/**
 * @brief Subtract one polynomial from another.
 *
 * POLYSUB(U,V) Returns the difference of two polynomials U-V. The arguments are
 * sequence of coefficients from highest order term to lowest, i.e., [1,2,3]
 * represents the polynomial X^2+2X+3.
 */
VectorXd polysub(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v);

/**
 * @brief Two dimensional convolution.
 *
 * C = CONV2(A, B) performs the 2-D convolution of matrices A and B. If [ma,na]
 * = size(A), [mb,nb] = size(B), and [mc,nc] = size(C), then mc =
 * max([ma+mb-1,ma,mb]) and nc = max([na+nb-1,na,nb]).
 */
MatrixXd conv2(
    const Eigen::Ref<const MatrixXd> P, const Eigen::Ref<const MatrixXd> S);

/**
 * @brief Convolution and polynomial multiplication.
 *
 * C = CONV(A, B) convolves vectors A and B.  The resulting vector is length
 * MAX([LENGTH(A)+LENGTH(B)-1,LENGTH(A),LENGTH(B)]). If A and B are vectors of
 * polynomial coefficients, convolving them is equivalent to multiplying the two
 * polynomials.
 */
VectorXd conv(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v);

/**
 * @brief Multiply polynomials.
 *
 * C = POLYMUL(A, B) returns the product of two polynomials A and B. This
 * function is an alias for CONV(A, B).
 */
VectorXd polymul(
    const Eigen::Ref<const VectorXd> u, const Eigen::Ref<const VectorXd> v);

/**
 * @brief Differentiate polynomial.
 *
 * POLYDER(A,B) returns the derivative of polynomial A*B.
 */
VectorXd polyder(
    const Eigen::Ref<const VectorXd> p, const Eigen::Ref<const VectorXd> q);

/**
 * @brief Differentiate polynomial.
 *
 * POLYDER(P) returns the derivative of the polynomial whose coefficients are
 * the elements of vector P.
 */
VectorXd polyder(const Eigen::Ref<const VectorXd> p);

/**
 * @brief Differentiate a 2D polynomial using Horner's method.
 *
 * POLYDER2D(C) returns the derivative of the polynomial whose coefficients are
 * the elements of matrix C.
 *
 * @param c @see polyval2d
 * @param axis Axis over which the derivative is taken. 0 for x axis and 1 for y
 * axis.
 */
MatrixXd polyder2d(const Eigen::Ref<const MatrixXd> c, int axis);

/**
 * @brief Integrate polynomial analytically.
 *
 * POLYINT(P,K) returns a polynomial representing the integral of polynomial P,
 * using a scalar constant of integration K.
 *
 */
VectorXd polyint(const Eigen::Ref<const VectorXd> p, double k = 0.0);

/**
 * @brief Fit polynomial to data.
 *
 * POLYFIT(X, Y, N) finds the coefficients of a polynomial P(X) of degree N that
 * fits the data Y best in a least-squares sense. P is a column vector of length
 * N+1 containing the polynomial coefficients in descending powers, P(1)*X^N +
 * P(2)*X^(N-1) +...+ P(N)*X + P(N+1).
 */
VectorXd polyfit(
    const Eigen::Ref<const VectorXd> x,
    const Eigen::Ref<const VectorXd> y,
    int n);

} // namespace crobot::math

