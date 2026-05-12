#pragma once
#include <Arduino.h>

namespace kisley {
namespace iris {

// Pure-data description of an iris stretcher's mechanical geometry.
// Defaults match the Kisley Lab v1.0 rig.
struct IrisGeometry {
  float r0     = 7.1f;     // mm — crank radius
  float rp     = 5.25f;    // mm — pin/link length contributor
  float Y0     = -6.1f;    // mm — must remain negative
  float rPin   = 0.5f;     // mm — pin offset for the expansion ratio
  float g      = 15.0f;    // gear ratio
  float maxEx  = 4.2f;     // safety clamp for Xgoto
  uint16_t pulsesPerRev = 1600; // microsteps per output revolution

  // Blade-arm radius in cm. Used to convert blade linear speed (cm/s) into
  // shaft angular speed (rad/s):  omega = bladeSpeed / bladeRadiusCm.
  float bladeRadiusCm = 13.6f;

  // E0 = r0 - rp (resting effective travel reference).
  float E0() const { return r0 - rp; }
};

} // namespace iris
} // namespace kisley
