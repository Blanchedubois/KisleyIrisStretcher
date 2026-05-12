#pragma once
#include "IrisGeometry.h"

namespace kisley {
namespace iris {

// Pure-math forward and inverse maps between crank angle θ and the
// expansion ratio Eₓ. Stateless; safe to call from anywhere.
//
// All work is done in double precision. Earlier code mixed float/double
// while using 1e-10 tolerances that fell below float epsilon (~1.2e-7);
// promoting to double makes those tolerances meaningful and is free on
// the ESP32-S3 FPU.
//
// Bidirectional Eₓ:
//   findTheta() handles Eₓ > 1 (CW), Eₓ < 1 (CCW), and Eₓ == 1 (center).
//   The kinematic model only naturally produces Eₓ > 1, so contraction
//   targets are mapped via mirror symmetry: solve for (2 − Eₓ) and
//   negate the result. See BIDIRECTIONAL_XGOTO.md §"Locked decisions".
class IrisKinematics {
public:
  // Forward map: θ (radians) → Eₓ (dimensionless expansion ratio).
  // Returns NaN if the geometry has no real solution at this θ.
  static double computeEx(const IrisGeometry& geo, double theta);

  // Inverse map: target Eₓ → θ (signed). Dispatches based on whether
  // the target is above, below, or equal to 1.0. Returns NaN if no
  // solution exists.
  static double findTheta(const IrisGeometry& geo, double targetEx);

  // Low-level inverse: solve only within an explicit θ bracket.
  // Bisection (50 iter) then Newton-Raphson (50 iter). Returns NaN if
  // the target is not bracketed.
  static double findThetaInBracket(const IrisGeometry& geo,
                                   double targetEx,
                                   double thetaLow,
                                   double thetaHigh);
};

} // namespace iris
} // namespace kisley
