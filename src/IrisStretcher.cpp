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
  const double angle = findTheta(targetEx);
  if (targetEx <= 1.001 || targetEx >= _geo.maxEx || isnan(angle)) {
    _io.println("Error: no valid \xce\xb8 found for that targetEx");
    return false;
  }
  _io.print("Computed \xce\xb8 (rad): ");
  _io.println(angle, 10);
  const double actualEx = computeEx(angle);
  _io.print("Resulting expansion: ");
  _io.println(actualEx, 10);
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
