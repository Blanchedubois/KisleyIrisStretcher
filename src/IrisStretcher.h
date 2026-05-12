#pragma once
#include <Arduino.h>
#include "IrisGeometry.h"
#include "IrisStepperDriver.h"
#include "IrisKinematics.h"

namespace kisley {
namespace iris {

// High-level facade combining geometry + kinematics + stepper.
//
// Typical lab use:
//   IrisStretcher stretcher(/*STEP=*/12, /*DIR=*/13);
//   stretcher.begin();
//   stretcher.gotoExpansion(1.5);
//
class IrisStretcher {
public:
  using CalibrationRoutine = void(*)(IrisStretcher& self, void* user);

  IrisStretcher(uint8_t stepPin,
                uint8_t dirPin,
                const IrisGeometry& geo = IrisGeometry{},
                Stream& io = Serial);

  void begin();

  // ---- High-level motion ----
  bool gotoExpansion(double targetEx);  // false if out of range or unreachable
  void goToZero();                      // drive to θ = 0
  void setZeroHere();                   // reset position counter to 0
  void calibrate();                     // runs default or registered routine
  void setBladeSpeed(double cmPerSec);  // updates step half-period

  // ---- Extension seams ----
  void setCalibrationRoutine(CalibrationRoutine cb, void* user = nullptr) {
    _calibCb = cb; _calibUser = user;
  }
  void onEachStep(IrisStepperDriver::StepCallback cb, void* user = nullptr) {
    _driver.onEachStep(cb, user);
  }
  void setStepLogging(bool enabled, uint16_t every = 1) {
    _driver.setStepLogging(&_io, enabled, every);
  }

  // ---- Low-level escape hatches ----
  void   rotateThetaRadians(double theta);
  long   currentSteps() const          { return _driver.currentPosition(); }
  double currentTheta() const          { return _driver.currentTheta(_geo); }
  double computeEx(double theta) const { return IrisKinematics::computeEx(_geo, theta); }
  double findTheta(double targetEx) const { return IrisKinematics::findTheta(_geo, targetEx); }

  // ---- Accessors ----
  const IrisGeometry& geometry() const { return _geo; }
  IrisGeometry&       geometry()       { return _geo; }
  IrisStepperDriver&  driver()         { return _driver; }
  Stream&             io()             { return _io; }

private:
  void defaultCalibration();

  IrisGeometry        _geo;
  IrisStepperDriver   _driver;
  Stream&             _io;
  CalibrationRoutine  _calibCb   = nullptr;
  void*               _calibUser = nullptr;
};

} // namespace iris
} // namespace kisley
