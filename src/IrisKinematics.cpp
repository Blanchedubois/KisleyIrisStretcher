#include "IrisKinematics.h"
#include <math.h>

namespace kisley {
namespace iris {

double IrisKinematics::computeEx(const IrisGeometry& geo, double Theta) {
  const double r0   = geo.r0;
  const double rp   = geo.rp;
  const double y0   = geo.Y0;
  const double rPin = geo.rPin;
  const double E0   = geo.E0();

  const double theta = Theta - (M_PI / 2.0);
  const double mu = (r0 * sin(theta) - y0) / (r0 * cos(theta));

  const double A = 1.0 + mu * mu;
  const double b = -2.0 * r0 * cos(theta)
                   - 2.0 * r0 * sin(theta) * mu
                   + 2.0 * mu * y0;
  const double c = r0 * r0
                   - 2.0 * r0 * sin(theta) * y0
                   + y0 * y0
                   - rp * rp;

  const double disc = b * b - 4.0 * A * c;
  if (disc < 0.0) return NAN;
  const double sqrt_disc = sqrt(disc);

  // Take the "minus" root, matching the original implementation.
  const double x = (-b - sqrt_disc) / (2.0 * A);

  const double underRoot = A * x * x + 2.0 * mu * y0 * x + y0 * y0;
  if (underRoot < 0.0) return NAN;

  return (sqrt(underRoot) - rPin) / (E0 - rPin);
}

double IrisKinematics::findTheta(const IrisGeometry& geo, double targetEx) {
  // Center: θ = 0 by definition.
  if (fabs(targetEx - 1.0) < 1e-9) return 0.0;

  if (targetEx > 1.0) {
    // Expansion: original CW bracket.
    return findThetaInBracket(geo, targetEx, 0.0001, 0.6);
  }

  // Contraction: mirror around 1.0 and negate. The kinematic model
  // doesn't produce Eₓ < 1 naturally — walking θ past the singularity
  // gives Eₓ values >> 1, not below. Convention: a CCW move by the
  // same angle magnitude that a CW move would use to reach (2 − Eₓ).
  const double mirroredTarget = 2.0 - targetEx;
  const double theta = findThetaInBracket(geo, mirroredTarget, 0.0001, 0.6);
  if (isnan(theta)) return NAN;
  return -theta;
}

double IrisKinematics::findThetaInBracket(const IrisGeometry& geo,
                                          double targetEx,
                                          double thetaLow,
                                          double thetaHigh) {
  // Bracket check.
  double fLow  = computeEx(geo, thetaLow)  - targetEx;
  double fHigh = computeEx(geo, thetaHigh) - targetEx;
  if (isnan(fLow) || isnan(fHigh)) return NAN;
  if (fLow * fHigh > 0.0) return NAN;

  // 50-iteration bisection to narrow.
  for (int i = 0; i < 50; i++) {
    const double mid  = 0.5 * (thetaLow + thetaHigh);
    const double fmid = computeEx(geo, mid) - targetEx;
    if (fLow * fmid <= 0.0) {
      thetaHigh = mid;
      fHigh = fmid;
    } else {
      thetaLow = mid;
      fLow = fmid;
    }
  }

  // 50-iteration Newton-Raphson with central-difference derivative.
  double theta = 0.5 * (thetaLow + thetaHigh);
  const double tolF     = 1e-10;
  const double tolTheta = 1e-10;
  const double eps      = 1e-8;

  for (int iter = 0; iter < 50; iter++) {
    const double f = computeEx(geo, theta) - targetEx;
    if (fabs(f) < tolF) break;

    const double f1 = computeEx(geo, theta + eps) - targetEx;
    const double f2 = computeEx(geo, theta - eps) - targetEx;
    const double fp = (f1 - f2) / (2.0 * eps);

    const double prevTheta = theta;
    const double dtheta    = -f / fp;
    theta += dtheta;

    if (isnan(theta) || isinf(theta)) {
      theta = prevTheta;
      break;
    }
    if (fabs(dtheta) < tolTheta) break;
  }

  return theta;
}

} // namespace iris
} // namespace kisley
