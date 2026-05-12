#include "IrisStretcher.h"
#include <math.h>

namespace kisley {
namespace iris {

IrisStretcher::IrisStretcher(uint8_t stepPin,
                             uint8_t dirPin,
                             const IrisGeometry& geo,
                             Stream& io)
  : _geo(geo),
    _driver(stepPin, dirPin),
    _io(io) {}

void IrisStretcher::begin() {
  _driver.begin();
}

void IrisStretcher::rotateThetaRadians(double theta) {
  const long delta = _driver.rotateThetaRadians(_geo, theta);
  _io.print("Moved ");
  _io.print(delta);
  _io.print(" steps, new position = ");
  _io.println(_driver.currentPosition());
}

bool IrisStretcher::gotoExpansion(double targetEx) {
  if (targetEx <= _geo.minEx || targetEx >= _geo.maxEx) {
    _io.print("Error: targetEx out of range [");
    _io.print(_geo.minEx, 3);
    _io.print(", ");
    _io.print(_geo.maxEx, 3);
    _io.println("]");
    return false;
  }

  // Center: short-circuit straight to θ=0 (back to the saved zero step).
  if (fabs(targetEx - 1.0) < 1e-6) {
    _io.println("Target is center (Ex=1.0); returning to \xce\xb8=0.");
    rotateThetaRadians(0.0);
    return true;
  }

  const double angle = findTheta(targetEx);
  if (isnan(angle)) {
    _io.println("Error: no valid \xce\xb8 found for that targetEx");
    return false;
  }

  _io.print("Target Ex: ");
  _io.print(targetEx, 6);
  _io.println(targetEx > 1.0 ? " (CW)" : " (CCW)");
  _io.print("Computed \xce\xb8 (rad): ");
  _io.println(angle, 10);

  // Forward-kinematic verification only roundtrips for the CW branch.
  // For CCW (mirror convention) we just echo the target.
  _io.print("Resulting expansion: ");
  if (targetEx > 1.0) {
    _io.println(computeEx(angle), 10);
  } else {
    _io.println(targetEx, 10);
  }

  rotateThetaRadians(angle);
  return true;
}

void IrisStretcher::goToZero() {
  const long startPos = _driver.currentPosition();
  _io.print("Current position (steps): ");
  _io.println(startPos);
  _io.print("Moving back to zero from ");
  _io.print(startPos);
  _io.println(" steps\xe2\x80\xa6");
  rotateThetaRadians(0.0);
  _io.println("Motor is now at zero position (\xce\xb8 = 0).");
}

void IrisStretcher::setZeroHere() {
  const long current = _driver.currentPosition();
  _io.print("Current position (steps): ");
  _io.println(current);
  _driver.setCurrentPosition(0);
  _io.print("Internal counter reset to zero. Previous position was: ");
  _io.println(current);
}

void IrisStretcher::calibrate() {
  if (_calibCb) {
    _calibCb(*this, _calibUser);
    return;
  }
  defaultCalibration();
}

void IrisStretcher::defaultCalibration() {
  _io.println("Starting calibration...");
  rotateThetaRadians(0.7);
  setZeroHere();
  _io.println("Moving back in by -0.290 rad...");
  rotateThetaRadians(-0.287);
  setZeroHere();
  _io.println("Setting Zero");
  setZeroHere();
  _io.println("Calibration complete.");
}

void IrisStretcher::setBladeSpeed(double cmPerSec) {
  const double omega = cmPerSec / _geo.bladeRadiusCm;
  unsigned long delayUs = 0;
  if (omega > 0.0) {
    delayUs = (unsigned long)lround(
      (M_PI * 1e6) / (omega * double(_geo.pulsesPerRev) * _geo.g));
  }
  _driver.setStepHalfPeriodUs(delayUs);
  _io.print("Step pulse half\xe2\x80\x90period delay set to ");
  _io.print(delayUs);
  _io.println(" \xc2\xb5s");
}

} // namespace iris
} // namespace kisley
