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

bool IrisStretcher::gotoExpansion(double signedEx) {
  // Convention: |signedEx| is the magnitude in [1.0, maxEx]. Sign of
  // signedEx selects rotation direction — positive = CW, negative = CCW.
  // Both directions cover the same range; CCW rotates by the same θ
  // magnitude as the matching CW value but in the opposite direction.
  const double magnitude = fabs(signedEx);

  // Center: |Ex| ≤ 1.0 means return to the zero step. Treat both 0 and
  // ±1.0 as "go to center" so the LCD's CCW path can also commit center.
  if (magnitude < 1.0 + 1e-6) {
    _io.println("Target is center; returning to \xce\xb8=0.");
    rotateThetaRadians(0.0);
    return true;
  }

  if (magnitude >= _geo.maxEx) {
    _io.print("Error: |Ex| out of range [1.0, ");
    _io.print(_geo.maxEx, 3);
    _io.println(")");
    return false;
  }

  const double posTheta = findTheta(magnitude);   // always positive
  if (isnan(posTheta)) {
    _io.println("Error: no valid \xce\xb8 found for that targetEx");
    return false;
  }

  const bool   cw = (signedEx >= 0.0);
  const double angle = cw ? posTheta : -posTheta;

  _io.print("Target Ex: ");
  _io.print(magnitude, 6);
  _io.println(cw ? " (CW)" : " (CCW)");
  _io.print("Computed \xce\xb8 (rad): ");
  _io.println(angle, 10);

  // Forward-kinematic verification only roundtrips for the CW branch
  // (the model has no Eₓ<1 domain). For CCW we just echo the magnitude.
  _io.print("Resulting expansion: ");
  if (cw) {
    _io.println(computeEx(angle), 10);
  } else {
    _io.println(magnitude, 10);
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
