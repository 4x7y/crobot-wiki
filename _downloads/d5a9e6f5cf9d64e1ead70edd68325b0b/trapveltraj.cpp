#include "crobot/planning/trapveltraj.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;

// //==============================================================================
// void crobot::planning::trapveltraj(
//     const std::vector<math::VectorXd>& wayPoints,
//     std::size_t num_samples,
//     std::vector<math::VectorXd>& q,
//     std::vector<math::VectorXd>& qd,
//     std::vector<math::VectorXd>& qdd,
//     std::vector<math::VectorXd> vel,
//     std::vector<math::VectorXd> acc,
//     std::vector<math::VectorXd> tFi,
//     std::vector<math::VectorXd> tAc,
//     std::vector<double>& t) {

//   // dimensions
//   std::size_t m = num_samples;
//   std::size_t p = wayPoints.size();
//   int n = wayPoints[0].size();

//   // Initialize outputs
//   q.resize(m, math::VectorXd(n));
//   qd.resize(m, math::VectorXd(n));
//   qdd.resize(m, math::VectorXd(n));

//   // Compute parameters, coefficients, and breaks, and store for subsequent
//   // use in pp-form and to generate outputs.
//   std::vector<math::MatrixXd> parameterMat
//       = std::vector<math::MatrixXd>(math::MatrixXd::Zero(n, 6), p - 1);

//   // Initialize the coefficient matrix to zeros. For P waypoints, there are
//   // (P-1 segments) x (N dimensions) of polynomials, each with four
//   // coefficients.
//   math::MatrixXd coeffMat(3 * (p - 1) * n, 3);
//   math::MatrixXd breakMat(n, 3 * (p - 1) + 1);
//   for (std::size_t i = 0; i < n; ++i) {
//     for (std::size_t j = 0; j < p - 1; ++j) {
//       // Compute the trapezoidal profile parameters for the profile
//       // segment between two waypoints. Since the user-specified
//       // parameter values may be empty here, pass the indices as
//       // inputs following function.
//       auto [segVel, segAcc, segATime, segFTime]
//           = planning::computeProfileParams(
//               i, j, way_woints, vel, acc, tFi, tAc);
//       parameterMat[j].block(i, 0, 1, 6) = math::Vector6d(
//           wayPoints[j](i),
//           wayPoints[j + 1](i),
//           segVel,
//           segAcc,
//           segATime,
//           segFTime);
//       auto [coefs, breaks] = computeScalarLSPBCoefficients(
//           wayPoints[j](i),
//           wayPoints[j + 1](i),
//           segVel,
//           segAcc,
//           segATime,
//           segFTime);

//       // There are three elements that are computed by the scalar
//       // coefficients, and they are entered as three rows in the
//       // coefficient matrix
//       coefIndex = false(3 * (p - 1) * n, 1);
//       lspbSegIndices
//           = (3 * (j - 1) * n + i) : n : (3 * (j - 1) * n + i + 2 * n);
//       coefIndex(lspbSegIndices) = true;

//       coeffMat(coefIndex, :) = coefs;
//       breakMat(i, (3 * j - 2)
//                : (3 * j + 1))
//           = breaks + breakMat(i, (3 * j - 2));
//     }
//   }

//   // Compute piecewise polynomial
//   hasMultipleBreaks = checkPolyForMultipleBreaks(breakMat);
//   [ breaksCell, coeffsCell ]
//       = processPolynomialResults(breakMat, coeffMat, hasMultipleBreaks, p);

//   // Create time vector and use it to evaluate piecewise polynomials
//   t = linspace(0, max(sum(parameterMat( :, :, 6), 2)), m);

//   // Define array sizes first so that cell may be defined in a single for
//   // loop for codegen compatibility
//   if (hasMultipleBreaks) {
//     numComputedPolynomials = n;
//     indivPolyDim = 1;
//   }
//   else {
//     numComputedPolynomials = 1;
//     indivPolyDim = n;
//   }

//   ppCell = cell(numComputedPolynomials, 1);
//   for (int jj = 1 : numComputedPolynomials) {
//     if (hasMultipleBreaks) {
//       rowSelection = jj;
//       cellSelection = jj;
//     }
//     else {
//       rowSelection = 1 : n;
//       cellSelection = 1;
//     }
//     [
//       q(rowSelection, 1
//         : m),
//       qd(rowSelection, 1
//          : m),
//       qdd(rowSelection, 1
//           : m),
//       ppCell{jj}
//     ]
//         = ... generateTrajectoriesFromCoefs(
//             breaksCell{cellSelection},
//             coeffsCell{cellSelection},
//             indivPolyDim,
//             t);
//   }
// }

// /**
//  * @brief Generate N-dimensional position, velocity, and acceleration
//  * trajectories
//  *
//  * Given a set of breaks and coefficients, the dimension N, and a 1xM time
//  * vector, T, this function computes the position vector, Q, as well as its
//  * derivatives, QD and QDD, as M-dimensional row-vectors in time.
//  */
// [ q, qd, qdd, pp ] = generateTrajectoriesFromCoefs(breaks, coeffs, dim, t) {
//   // Add flat segments to the start and end of each trajectory so the
//   // values will be held constant outside the user-specified time values
//   [ modBreaks, modCoeffs ]
//       = robotics.core.internal.addFlatSegmentsToPPFormParts(
//           breaks, coeffs, dim);

//   dCoeffs = robotics.core.internal.polyCoeffsDerivative(modCoeffs);
//   ddCoeffs = robotics.core.internal.polyCoeffsDerivative(dCoeffs);

//   pp = mkpp(modBreaks, modCoeffs, dim);
//   ppd = mkpp(modBreaks, dCoeffs, dim);
//   ppdd = mkpp(modBreaks, ddCoeffs, dim);

//   coder.varsize('q', 'qd', 'qdd');

//   q = ppval(pp, t);
//   qd = ppval(ppd, t);
//   qdd = ppval(ppdd, t);
// }

//==============================================================================
std::tuple<double, double, double, double>
crobot::planning::computeProfileParams(
    std::size_t i,
    std::size_t j,
    const std::vector<math::VectorXd>& wayPoints,
    std::optional<std::vector<math::VectorXd>> Vel,
    std::optional<std::vector<math::VectorXd>> Acc,
    std::optional<std::vector<math::VectorXd>> TFi,
    std::optional<std::vector<math::VectorXd>> TAc) {

  // Initialize outputs
  double aParam  = 0.;
  double vParam  = 0.;
  double tAParam = 0.;
  double tFParam = 0.;

  // Compute the initial and final profile segment position, scalars s0 and sF.
  double s0        = wayPoints[j](i);
  double sF        = wayPoints[j + 1](i);
  double deltaSign = 1;

  // If(sF - s0) is negative, swap them while computing terms, and pass sign
  // to the output .This ensures that the upper and lower bound inequalities can
  // be computed without special case evaluation.
  if (sF < s0) {
    std::swap(s0, sF);
    deltaSign = -1;
  }

  // Compute velocity, acceleration, acceleration time, and final time
  // based on provided inputs. To ensure a valid profile can be found, the
  // values for the product of velocity and final time, v*TF, must fall
  // between (sF-s0) and 2*(sF - s0). When a user specifies less than two
  // inputs, values are chosen so that v*tF is exactly in the middle of
  // the allowable bounds. Convert from binary to decimal for evaluation.
  // This is done manually to ensure codegen compatibility.
  math::Vector4i inputCombo(
      Vel.has_value() ? 1 : 0,
      Acc.has_value() ? 1 : 0,
      TFi.has_value() ? 1 : 0,
      TAc.has_value() ? 1 : 0);
  int inputCombination = inputCombo.transpose() * math::Vector4i(8, 4, 2, 1);

  // Initialize variables to non-empty values for code generation
  // compatibility
  double VelSwitch = inputCombo(0) ? Vel.value()[j](i) : 1.0;
  double AccSwitch = inputCombo(1) ? Acc.value()[j](i) : 1.0;
  double TFiSwitch = inputCombo(2) ? TFi.value()[j](i) : 1.0;
  double TAcSwitch = inputCombo(3) ? TAc.value()[j](i) : 1.0;

  double b2_4ac, root1, root2;

  switch (inputCombination) {
    case 12:
      // Velocity and Acceleration
      vParam = VelSwitch;
      aParam = AccSwitch;
      // Solve for acceleration time and final time
      tAParam = vParam / aParam;
      tFParam = (vParam * tAParam + sF - s0) / vParam;
      break;
    case 10:
      // Velocity and Final Time
      vParam  = VelSwitch;
      tFParam = TFiSwitch;
      // Solve for acceleration time and acceleration
      tAParam = (s0 - sF + vParam * tFParam) / vParam;
      aParam  = vParam / tAParam;
      break;
    case 9:
      // Velocity and Acceleration Time
      vParam  = VelSwitch;
      tAParam = TAcSwitch;
      // Solve for acceleration and final time
      aParam  = vParam / tAParam;
      tFParam = (vParam * tAParam + sF - s0) / vParam;
      break;
    case 8:
      // Velocity only
      // Choose the final time so that v*TF is in the middle of the
      // acceptable range.
      vParam  = VelSwitch;
      tFParam = (1.5 * (sF - s0) / vParam);
      // Solve for acceleration time and acceleration
      tAParam = (s0 - sF + vParam * tFParam) / vParam;
      aParam  = vParam / tAParam;
      break;
    case 6:
      // Acceleration and Final Time
      aParam  = AccSwitch;
      tFParam = TFiSwitch;
      // Solve for acceleration time and velocity
      b2_4ac = aParam * (aParam * tFParam * tFParam + 4. * s0 - 4. * sF);
      // Ensure real solution exist
      CROBOT_ASSERT_ERROR(b2_4ac >= 0.0);
      root1 = (aParam * tFParam - std::sqrt(b2_4ac)) / (2. * aParam);
      root2 = (aParam * tFParam + std::sqrt(b2_4ac)) / (2. * aParam);
      // Ensure positive root exist
      CROBOT_ASSERT_ERROR(std::max(root1, root2) >= 0.0);
      tAParam = root1 > 0.0 ? root1 : root2;
      // Choose the smaller over the larger option if both remain
      vParam = (sF - s0) / (tFParam - tAParam);
      break;
    case 5:
      // Acceleration and Acceleration Time
      aParam  = AccSwitch;
      tAParam = TAcSwitch;

      // Solve for velocity and final time
      vParam  = aParam * tAParam;
      tFParam = (vParam * tAParam + sF - s0) / vParam;
      break;
    case 4:
      // Acceleration only.
      // Since the conditions apply bounds to the value of v*TF, choose
      // v*TF to be the average value, 1.5*(sF-s0), by solving for
      // acceleration time (tA) from the equations v*TF = sF-s0+v*TA and v =
      // a*tA.
      aParam  = AccSwitch;
      tAParam = sqrt((sF - s0) / (2. * aParam));

      // Solve for velocity and final time
      vParam  = aParam * tAParam;
      tFParam = (sF - s0 + aParam * tAParam * tAParam) / vParam;
      break;
    case 3:
      // Final Time and Acceleration Time
      tFParam = TFiSwitch;
      tAParam = TAcSwitch;

      // Solve for velocity and acceleration
      vParam = (sF - s0) / (tFParam - tAParam);
      aParam = vParam / tAParam;
      break;
    case 2:
      // Final Time only
      // Choose the velocity to be in the middle of the acceptable range.
      tFParam = TFiSwitch;
      // Choose V for the middle of the range
      vParam = 1.5 * (sF - s0) / tFParam;

      // Solve for acceleration time and acceleration
      tAParam = (s0 - sF + vParam * tFParam) / vParam;
      aParam  = vParam / tAParam;
      break;
    case 1:
      // Acceleration Time only
      // Since the conditions apply bounds to the value of v*TF, choose
      // v*TF to be the average value, 1.5*(sF-s0), by solving for
      // acceleration (a) from the equations v*TF = sF - s0 + v*tA and
      // v = a*tA.
      tAParam = TAcSwitch;
      aParam  = (sF - s0) / (2. * tAParam * tAParam);

      // Solve for velocity and final time
      vParam  = aParam * tAParam;
      tFParam = (sF - s0 + aParam * tAParam * tAParam) / vParam;
      break;
    case 0:
      // None
      // Assume final time is 1 and choose velocity, v, to be in the
      // middle of the acceptable range.
      tFParam = 1.;
      vParam  = 1.5 * (sF - s0) / tFParam;

      // Solve for acceleration time and acceleration
      tAParam = (s0 - sF + vParam * tFParam) / vParam;
      aParam  = vParam / tAParam;
      break;
  }

  // Handle edge cases and verify that meaningful profile will be produced.
  // Generate an error if the minimum conditions are not met
  if (s0 == sF) {
    // This will generate a flat profile. The only meaningful output
    // parameter is the final time. The acceleration time must be set
    // to be consistent with the general case.
    aParam = 0.;
    vParam = 0.;
    if (std::isnan(tFParam) || (tFParam == 0.0)) {
      // Depending on the input, it may or may not be possible to compute
      // tF. This check ensures that a valid value is used.
      tFParam = 1;
    }
    tAParam = tFParam / 3.;
  }
  else {
    CROBOT_ASSERT_ERROR(
        (sF - s0) / tFParam < vParam,
        "The selected parameter combination for wayPoints(1,1:2) must satisfy "
        "the following constraint: (q(end)-q(0))/EndTime must be less than "
        "Velocity.");
    CROBOT_ASSERT_ERROR(
        vParam <= 2. * (sF - s0) / tFParam,
        "The selected parameter combination for wayPoints(1,1:2) must satisfy "
        "the following constraint: Velocity*EndTime/2 must be less than or "
        "equal to q(end)-q(0).");
  }

  // Apply the sign to the output
  vParam = deltaSign * vParam;
  aParam = deltaSign * aParam;
  return std::make_tuple<>(vParam, aParam, tAParam, tFParam);
}

//==============================================================================
std::pair<math::Matrix3d, math::Vector4d>
crobot::planning::computeScalarLSPBCoefficients(
    double s0, double sF, double v, double a, double ta, double tf) {
  math::Vector4d breaks(0.0, ta, tf - ta, tf);
  math::Matrix3d coefs = math::Matrix3d::Zero();

  if (v == 0) {
    // Flat profile
    coefs.block<3, 1>(0, 2) = s0 * math::Vector3d::Ones();
  }
  else {
    // Section 1: From t=0 to t=TAccel
    coefs.block<1, 3>(0, 0) = math::Vector3d(a / 2., 0., s0).transpose();

    // Section 2: From t=TAccel to t=TFinal-TAccel
    coefs.block<1, 3>(1, 0)
        = math::Vector3d(0., v, a / 2. * ta * ta + s0).transpose();

    // Section 3: From t=TFinal-TAccel to t=TFinal
    coefs.block<1, 3>(2, 0)
        = math::Vector3d(-a / 2., v, sF + a / 2. * ta * ta - v * ta);
  }

  return std::make_pair<>(coefs, breaks);
}
