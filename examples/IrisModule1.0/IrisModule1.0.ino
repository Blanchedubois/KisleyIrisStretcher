// IrisModule1.0 — parity example for the KisleyIrisStretcher library.
//
// Drop-in replacement for the original monolithic IrisModule1.0.ino.
// Hardware: ESP32-S3, HD44780 I2C LCD, rotary encoder, 3 buttons,
// STEP/DIR stepper driver, optional NAU7802 strain gauge.

#include <USB.h>
#include <KisleyIrisStretcher.h>

using namespace kisley::iris;

// ---- Hardware pin map (matches original IrisModule1.0.ino) ----
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

IrisStretcher stretcher(PIN_STEP, PIN_DIR);

IrisMenuUI ui(stretcher, IrisMenuUI::Pins{
  PIN_MENU, PIN_DOWN, PIN_ACCEPT,
  PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW,
  PIN_SDA, PIN_SCL
});

IrisSerialConsole console(stretcher);

void setup() {
  delay(500);
  USB.begin();
  Serial.begin(115200);
  while (!Serial) {}

  stretcher.begin();
  stretcher.setStepLogging(true);    // matches strainData=true from V1

  // Override default about info with the sketch's build date/time so the
  // Xabout LCD page reports when *this* sketch was compiled, not the lib.
  IrisAboutInfo info;
  info.fwName    = "Iris Stretcher";
  info.fwVersion = "v1.0.0";
  info.labLine1  = "Kisley Lab";
  info.labLine2  = "CWRU";
  info.creator   = "Tejasvin Shrikanth";
  info.buildDate = __DATE__;
  info.buildTime = __TIME__;
  ui.setAboutInfo(info);

  ui.setInvertEncoder(false);
  ui.setUseInternalPulldown(true);
  ui.begin();

  console.setBannerLine("|KisleyLab V2.1 |");
  console.begin();
}

void loop() {
  ui.update();
  console.update();
}
