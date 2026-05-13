// IrisModule1.0 — canonical Kisley rig firmware.
//
// Drop-in replacement for the original monolithic IrisModule1.0.ino,
// plus:
//   - 9-channel NAU7802 strain array readout (IrisStrainArray)
//   - "Experiments" LCD submenu with Exp1 (1x → 3.4x CW → 1x) registered
//     and ready to run
//   - Serial commands Xstrain (continuous streaming) / Xrun <name> /
//     Xabort for host-driven experiment control
//
// Hardware: ESP32-S3, HD44780 I2C LCD, rotary encoder, 3 buttons,
// STEP/DIR stepper driver, 9 NAU7802 strain ADCs behind two TCA9548A
// muxes (5 on 0x71 ch0..4, 4 on 0x73 ch0..3).

#include <USB.h>
#include <KisleyIrisStretcher.h>

using namespace kisley::iris;

// ---- Hardware pin map ----
constexpr uint8_t PIN_STEP   = 12;
constexpr uint8_t PIN_DIR    = 13;

constexpr uint8_t PIN_MENU   = A0;
constexpr uint8_t PIN_DOWN   = A1;
constexpr uint8_t PIN_ACCEPT = A2;
constexpr uint8_t PIN_ENC_SW = A3;
constexpr uint8_t PIN_ENC_B  = A4;
constexpr uint8_t PIN_ENC_A  = A5;
constexpr uint8_t PIN_SDA    = 3;
constexpr uint8_t PIN_SCL    = 4;

// ---- Core library components ----
IrisStretcher     stretcher(PIN_STEP, PIN_DIR);
IrisStrainArray   strain;
IrisMenuUI        ui(stretcher, IrisMenuUI::Pins{
                    PIN_MENU, PIN_DOWN, PIN_ACCEPT,
                    PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW,
                    PIN_SDA, PIN_SCL});
IrisSerialConsole    console(stretcher);
IrisExperimentRunner runner(stretcher, strain);

// =====================================================================
// EXPERIMENTS — add new entries here and register them in setup().
// Each `targets` array holds signed Ex values (positive = CW, negative
// = CCW, |Ex| ≤ 1 returns to center). Hold time between waypoints is
// the runner's global setting (default 2000 ms).
// =====================================================================

const float kExp1Targets[] = { 1.0f, 3.4f, 1.0f };
const IrisExperiment kExp1 = { "Exp1", kExp1Targets, 3 };

void setup() {
  delay(500);
  USB.begin();
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  stretcher.begin();
  stretcher.setStepLogging(false);   // step spam disabled — Xstrain is the data path

  IrisAboutInfo info;
  info.fwName    = "Iris Stretcher";
  info.fwVersion = "v2.0.0";
  info.labLine1  = "Kisley Lab";
  info.labLine2  = "CWRU";
  info.creator   = "Tejasvin Shrikanth";
  info.buildDate = __DATE__;
  info.buildTime = __TIME__;
  ui.setAboutInfo(info);

  ui.setInvertEncoder(false);
  ui.setUseInternalPulldown(true);
  ui.begin();                        // brings up Wire (SDA=3, SCL=4) + LCD

  strain.begin();                    // discovers 9 NAU7802s, runs host-side tare

  // Register experiments here. Each call appends to the runner's list
  // and (via attachRunner below) makes it appear in the LCD submenu.
  runner.registerExperiment(kExp1);

  // Tell the UI and serial console about the runner so the
  // "Experiments" menu and Xstrain/Xrun/Xabort serial commands work.
  ui.attachRunner(runner);
  console.attachRunner(runner);

  console.setBannerLine("|KisleyLab V2.1 |");
  console.begin();
}

void loop() {
  ui.update();
  console.update();
  runner.update();   // drives experiment state machine + strain streaming
}
