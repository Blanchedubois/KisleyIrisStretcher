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
class IrisKinematics {
public:
  // Forward map: θ (radians) → Eₓ (dimensionless expansion ratio).
  static double computeEx(const IrisGeometry& geo, double theta);

  // Inverse map: target Eₓ → θ. Bisection (50 iters) then Newton-Raphson
  // (50 iters) over [thetaLow, thetaHigh] in radians. Returns NaN if no
  // bracket exists in that interval.
  static double findTheta(const IrisGeometry& geo,
                          double targetEx,
                          double thetaLow  = 0.0001,
                          double thetaHigh = 0.6);
};

} // namespace iris
} // namespace kisley
