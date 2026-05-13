#include "IrisMenuUI.h"
#include "IrisExperiment.h"
#include <math.h>

namespace kisley {
namespace iris {

namespace {

constexpr int LCD_COLS = 16;
constexpr int LCD_ROWS = 2;

struct HelpFrame { const char* a; const char* b; uint16_t ms; };

const HelpFrame kHelpFrames[] = {
  {"Commands:",    "________",     1200},
  {"1.Xgoto <v>",  "  move to Ex", 1200},
  {"2.Xzero",      "  moves to 1x",1200},
  {"3.XsetZero",   "  Ex=1x here", 1200},
  {"4.Xcalibrate", "  calibrates", 1200},
  {"5.Xspeed <v>", "  cm/s",       1200},
  {"6.Xhelp",      "  this msg",   1200},
};

constexpr uint8_t kHelpFrameCount = sizeof(kHelpFrames) / sizeof(kHelpFrames[0]);
constexpr uint8_t kAboutFrameCount = 7;

float roundToStep(float x, float st) {
  return (float)((long)lround(x / st)) * st;
}

} // namespace

IrisMenuUI::IrisMenuUI(IrisStretcher& stretcher, const Pins& pins, Stream& io)
  : _stretcher(stretcher),
    _io(io),
    _pins(pins),
    _btnMenu(pins.btnMenu),
    _btnDown(pins.btnDown),
    _btnAccept(pins.btnAccept),
    _btnEncSW(pins.encSW) {}

void IrisMenuUI::begin() {
  Wire.begin(_pins.sda, _pins.scl);

  _io.println("Initializing LCD with library auto-detection...");
  int status = _lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) {
    _io.print("LCD initialization failed! Status: ");
    _io.println(status);
    _io.println("Check I2C connections and LCD address");
  } else {
    _io.println("LCD initialized successfully!");
  }
  delay(50);
  _lcd.clear();
  delay(50);
  _lcd.backlight();
  delay(100);
  _lcd.clear();
  delay(50);

  // Identify the LCD address after init so the scan doesn't interfere.
  _io.println("Scanning I2C bus to identify LCD address...");
  _lcdAddress = scanI2CForLCD();
  if (_lcdAddress == 0x00) {
    _io.println("WARNING: No I2C LCD detected in scan");
  }

  // Buttons
  const uint8_t mode = _useInternalPulldown ? INPUT_PULLDOWN : INPUT;
  _btnMenu.begin(mode, true);
  _btnDown.begin(mode, true);
  _btnAccept.begin(mode, true);
  _btnEncSW.begin(mode, true);

  // Encoder
  _enc.begin(_pins.encA, _pins.encB);

  // Splash screen
  if (_showSplash) {
    _lcd.clear();
    delay(10);
    _lcd.setCursor(0, 0); _lcd.print(_about.fwName);
    _lcd.setCursor(0, 1); _lcd.print(_about.labLine1);
    _io.println("Splash screen displayed");
    delay(1500);

    if (_lcdAddress != 0x00) {
      _lcd.clear();
      _lcd.setCursor(0, 0);
      _lcd.print("LCD @ 0x");
      _lcd.print(_lcdAddress, HEX);
      _lcd.setCursor(0, 1);
      _lcd.print("I2C detected!");
      delay(1000);
    }
  }

  // Seed built-in menu items in the original order.
  _items[0] = {"XsetZero",    BuiltinKind::XsetZero,    nullptr, nullptr};
  _items[1] = {"Xzero",       BuiltinKind::Xzero,       nullptr, nullptr};
  _items[2] = {"Xcalibrate",  BuiltinKind::Xcalibrate,  nullptr, nullptr};
  _items[3] = {"Xspeed",      BuiltinKind::Xspeed,      nullptr, nullptr};
  _items[4] = {"Xgoto",       BuiltinKind::Xgoto,       nullptr, nullptr};
  _items[5] = {"Xhelp",       BuiltinKind::Xhelp,       nullptr, nullptr};
  _items[6] = {"Xabout",      BuiltinKind::Xabout,      nullptr, nullptr};
  _items[7] = {"Experiments", BuiltinKind::Experiments, nullptr, nullptr};
  _itemCount = 8;

  drawMenu();
}

uint8_t IrisMenuUI::scanI2CForLCD() {
  _io.println("Scanning I2C bus for LCD...");
  uint8_t foundAddress = 0x00;
  uint8_t deviceCount = 0;

  for (uint8_t address = 0x20; address <= 0x3F; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      _io.print("I2C device found at address 0x");
      if (address < 16) _io.print("0");
      _io.print(address, HEX);
      _io.println();
      if (address == 0x27 || address == 0x3F) {
        foundAddress = address;
        _io.print("  -> LCD detected at 0x");
        if (address < 16) _io.print("0");
        _io.println(address, HEX);
      } else if (foundAddress == 0x00) {
        foundAddress = address;
      }
      deviceCount++;
    } else if (error == 4) {
      _io.print("Unknown error at address 0x");
      if (address < 16) _io.print("0");
      _io.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    _io.println("No I2C devices found!");
  } else {
    _io.print("Found ");
    _io.print(deviceCount);
    _io.println(" I2C device(s)");
  }
  if (foundAddress != 0x00) {
    _io.print("Using LCD address: 0x");
    if (foundAddress < 16) _io.print("0");
    _io.println(foundAddress, HEX);
  }
  return foundAddress;
}

bool IrisMenuUI::registerMenuItem(const char* label,
                                  ActionCallback cb,
                                  void* user) {
  if (_itemCount >= MAX_MENU_ITEMS) return false;
  _items[_itemCount++] = {label, BuiltinKind::Custom, cb, user};
  if (_ui == UiState::MENU) drawMenu();
  return true;
}

void IrisMenuUI::attachRunner(IrisExperimentRunner& runner) {
  _runner = &runner;
}

// ---- Drawing ----

void IrisMenuUI::drawMenu() {
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.write(byte(0x7E));            // right-arrow caret
  _lcd.print(_items[_menuIndex].label);
  _lcd.setCursor(0, 1);
  _lcd.print(' ');
  const uint8_t next = (_menuIndex + 1) % _itemCount;
  _lcd.print(_items[next].label);
}

void IrisMenuUI::drawStatus(const char* msg) {
  _lcd.clear();
  _lcd.setCursor(0, 0); _lcd.print(msg);
  _lcd.setCursor(0, 1); _lcd.print("Running...");
}

void IrisMenuUI::printFloatFixed(float v, uint8_t places) {
  long scale = 1;
  for (uint8_t i = 0; i < places; i++) scale *= 10;
  long s  = lround(v * scale);
  long ip = s / scale;
  long fp = labs(s % scale);
  _lcd.print(ip);
  _lcd.print('.');
  long div = scale / 10;
  for (uint8_t i = 1; i < places; i++) {
    if (fp < div) _lcd.print('0');
    div /= 10;
  }
  _lcd.print(fp);
}

void IrisMenuUI::drawEditXspeed() {
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("Xspeed:");
  _lcd.print(_xSpeedFineMode ? " F" : " C");
  _lcd.setCursor(0, 1);
  printFloatFixed(_xSpeedValue, _xSpeedFineMode ? 2 : 1);
  _lcd.print(" [SW]");
}

void IrisMenuUI::drawExperimentsMenu() {
  _lcd.clear();
  if (!_runner || _runner->experimentCount() == 0) {
    _lcd.setCursor(0, 0); _lcd.print("Experiments:");
    _lcd.setCursor(0, 1); _lcd.print("(none yet)");
    return;
  }
  const uint8_t n = _runner->experimentCount();
  if (_expIndex >= n) _expIndex = 0;
  const IrisExperiment* cur  = _runner->experiment(_expIndex);
  const IrisExperiment* next = _runner->experiment((_expIndex + 1) % n);
  _lcd.setCursor(0, 0);
  _lcd.write(byte(0x7E));
  if (cur) _lcd.print(cur->name);
  _lcd.setCursor(0, 1);
  _lcd.print(' ');
  if (next) _lcd.print(next->name);
}

void IrisMenuUI::drawRunningExperiment() {
  if (!_runner) return;
  const IrisExperiment* exp = _runner->currentExperiment();
  _lcd.clear();
  _lcd.setCursor(0, 0);
  if (exp) {
    _lcd.print(exp->name);
    _lcd.print(' ');
    _lcd.print(_runner->currentStepIndex() + 1);
    _lcd.print('/');
    _lcd.print(exp->nTargets);
  } else {
    _lcd.print("(no exp)");
  }
  _lcd.setCursor(0, 1);
  const float t = _runner->currentTargetEx();
  const float mag = fabsf(t);
  // Format "X.XX <CW|CCW> <M|H>"
  printFloatFixed(mag, 2);
  if (mag > 1.0f + 1e-4f) _lcd.print(t >= 0 ? " CW  " : " CCW ");
  else                    _lcd.print("     ");
  _lcd.print(_runner->isHolding() ? 'H' : 'M');
}

void IrisMenuUI::drawEditXgoto() {
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("Xgoto:");
  _lcd.print(_xGotoFineMode ? " F" : " C");
  _lcd.setCursor(0, 1);
  printFloatFixed(_xGotoValue, _xGotoFineMode ? 3 : 2);
  _lcd.print(_xGotoDirCw ? " CW  " : " CCW ");
  _lcd.print("[SW]");
}

// ---- Help / About paging ----

void IrisMenuUI::enterHelp() {
  _ui = UiState::HELP_SCROLL;
  _scrollFrame = 0;
  _scrollNextMs = millis();   // draw first frame on next tick
  tickHelp();
}

void IrisMenuUI::tickHelp() {
  if (_scrollFrame >= kHelpFrameCount) {
    _ui = UiState::MENU;
    drawMenu();
    return;
  }
  const HelpFrame& f = kHelpFrames[_scrollFrame];
  _lcd.clear();
  _lcd.setCursor(0, 0); _lcd.print(f.a);
  _lcd.setCursor(0, 1); _lcd.print(f.b);
  _scrollNextMs = millis() + f.ms;
  _scrollFrame++;
}

void IrisMenuUI::enterAbout() {
  _ui = UiState::ABOUT_SCROLL;
  _scrollFrame = 0;
  _scrollNextMs = millis();
  tickAbout();
}

void IrisMenuUI::tickAbout() {
  if (_scrollFrame >= kAboutFrameCount) {
    _ui = UiState::MENU;
    drawMenu();
    return;
  }
  _lcd.clear();
  uint16_t holdMs = 900;
  switch (_scrollFrame) {
    case 0: // FW name + version
      _lcd.setCursor(0, 0); _lcd.print(_about.fwName);
      _lcd.setCursor(0, 1); _lcd.print("FW "); _lcd.print(_about.fwVersion);
      holdMs = 1000;
      break;
    case 1: // Build date
      _lcd.setCursor(0, 0); _lcd.print("Build:");
      _lcd.setCursor(0, 1); _lcd.print(_about.buildDate);
      break;
    case 2: // Build time
      _lcd.setCursor(0, 0); _lcd.print("Time:");
      _lcd.setCursor(0, 1); _lcd.print(_about.buildTime);
      break;
    case 3: // Creator
      _lcd.setCursor(0, 0); _lcd.print("Creator:");
      _lcd.setCursor(0, 1); _lcd.print(_about.creator);
      holdMs = 1000;
      break;
    case 4: // Lab lines
      _lcd.setCursor(0, 0); _lcd.print(_about.labLine1);
      _lcd.setCursor(0, 1); _lcd.print(_about.labLine2);
      break;
    case 5: // Encoder direction
      _lcd.setCursor(0, 0); _lcd.print("Encoder dir:");
      _lcd.setCursor(0, 1); _lcd.print(_invertEncoder ? "inverted" : "normal");
      holdMs = 800;
      break;
    case 6: // Pulldown mode
      _lcd.setCursor(0, 0); _lcd.print("Buttons:");
      _lcd.setCursor(0, 1); _lcd.print(_useInternalPulldown ? "INT pulldown" : "EXT pulldown");
      holdMs = 800;
      break;
  }
  _scrollNextMs = millis() + holdMs;
  _scrollFrame++;
}

// ---- Action dispatch ----

void IrisMenuUI::runCurrentItem() {
  const MenuEntry& e = _items[_menuIndex];
  switch (e.kind) {
    case BuiltinKind::XsetZero:
      drawStatus("XsetZero");
      _stretcher.setZeroHere();
      _statusUntilMs = millis() + 1000;
      _ui = UiState::RUNNING;
      break;
    case BuiltinKind::Xzero:
      drawStatus("Xzero");
      _stretcher.goToZero();
      _statusUntilMs = millis() + 1000;
      _ui = UiState::RUNNING;
      break;
    case BuiltinKind::Xcalibrate:
      drawStatus("Xcalibrate");
      _stretcher.calibrate();
      _statusUntilMs = millis() + 1000;
      _ui = UiState::RUNNING;
      break;
    case BuiltinKind::Xspeed:
      _ui = UiState::EDIT_XSPEED;
      drawEditXspeed();
      break;
    case BuiltinKind::Xgoto:
      _ui = UiState::EDIT_XGOTO;
      drawEditXgoto();
      break;
    case BuiltinKind::Xhelp:
      enterHelp();
      break;
    case BuiltinKind::Xabout:
      enterAbout();
      break;
    case BuiltinKind::Experiments:
      if (!_runner || _runner->experimentCount() == 0) {
        drawStatus("No experiments");
        _statusUntilMs = millis() + 1200;
        _ui = UiState::RUNNING;
      } else {
        _expIndex = 0;
        _ui = UiState::EXPERIMENTS_MENU;
        drawExperimentsMenu();
      }
      break;
    case BuiltinKind::Custom:
      if (e.callback) {
        drawStatus(e.label);
        e.callback(*this, e.user);
        _statusUntilMs = millis() + 1000;
        _ui = UiState::RUNNING;
      }
      break;
  }
}

// ---- Main update loop ----

void IrisMenuUI::update() {
  int8_t ed = _enc.step();
  if (_invertEncoder) ed = -ed;

  // Cache button edges ONCE per update. Fixes bug 7.1 from
  // IMPLEMENTATION.md: the prior code called .fell() twice per branch,
  // and edge() consumes the transition on first call.
  const bool menuPressed   = _btnMenu.fell();
  const bool downPressed   = _btnDown.fell();
  const bool acceptPressed = _btnAccept.fell();
  const bool encSWPressed  = _btnEncSW.fell();

  switch (_ui) {
    case UiState::MENU: {
      if (ed == +1) {
        _menuIndex = (_menuIndex + 1) % _itemCount;
        drawMenu();
      } else if (ed == -1) {
        _menuIndex = (_menuIndex + _itemCount - 1) % _itemCount;
        drawMenu();
      }
      if (downPressed) {
        _menuIndex = (_menuIndex + 1) % _itemCount;
        drawMenu();
      }
      if (acceptPressed) runCurrentItem();
      if (menuPressed) {
        _lcd.noBacklight();
        delay(60);
        _lcd.backlight();
      }
    } break;

    case UiState::RUNNING:
      if (millis() >= _statusUntilMs) {
        _ui = UiState::MENU;
        drawMenu();
      }
      break;

    case UiState::HELP_SCROLL:
      if (millis() >= _scrollNextMs) tickHelp();
      if (acceptPressed || menuPressed) {
        _ui = UiState::MENU;
        drawMenu();
      }
      break;

    case UiState::ABOUT_SCROLL:
      if (millis() >= _scrollNextMs) tickAbout();
      if (acceptPressed || menuPressed) {
        _ui = UiState::MENU;
        drawMenu();
      }
      break;

    case UiState::EDIT_XSPEED: {
      if (encSWPressed) {
        _xSpeedFineMode = !_xSpeedFineMode;
        drawEditXspeed();
      }
      const float step = _xSpeedFineMode ? xSpeedStepFine : xSpeedStepCoarse;
      if (ed == +1) {
        _xSpeedValue = constrain(roundToStep(_xSpeedValue + step, step),
                                 xSpeedMin, xSpeedMax);
        drawEditXspeed();
      } else if (ed == -1) {
        _xSpeedValue = constrain(roundToStep(_xSpeedValue - step, step),
                                 xSpeedMin, xSpeedMax);
        drawEditXspeed();
      }
      if (acceptPressed && !_btnDown.isPressed()) {
        drawStatus("Xspeed set");
        _stretcher.setBladeSpeed(_xSpeedValue);
        _statusUntilMs = millis() + 800;
        _ui = UiState::RUNNING;
      }
      if (menuPressed) {
        _ui = UiState::MENU;
        drawMenu();
      }
    } break;

    case UiState::EDIT_XGOTO: {
      if (encSWPressed) {
        _xGotoFineMode = !_xGotoFineMode;
        drawEditXgoto();
      }
      if (downPressed) {
        _xGotoDirCw = !_xGotoDirCw;
        drawEditXgoto();
      }
      const float step = _xGotoFineMode ? xGotoStepFine : xGotoStepCoarse;
      const float goMax = _stretcher.geometry().maxEx;
      // Treat the encoder as a single linear axis:
      //   maxEx CCW ←──── 1.0 ────→ maxEx CW
      // ed = +1 moves position toward CW (right on the axis).
      // ed = -1 moves position toward CCW (left on the axis).
      // Magnitude is never shown below 1.0 — the direction tag flips
      // when the position crosses through center.
      if (ed != 0) {
        const bool outward =
          (_xGotoDirCw && ed == +1) || (!_xGotoDirCw && ed == -1);
        if (outward) {
          // Same direction as current side: move away from center.
          _xGotoValue = constrain(roundToStep(_xGotoValue + step, step),
                                  1.0f, goMax);
        } else {
          // Moving toward center; if at the floor, cross over and flip.
          if (_xGotoValue > 1.0f + step / 2.0f) {
            _xGotoValue = roundToStep(_xGotoValue - step, step);
            if (_xGotoValue < 1.0f) _xGotoValue = 1.0f;
          } else {
            _xGotoDirCw = !_xGotoDirCw;
            _xGotoValue = constrain(roundToStep(1.0f + step, step),
                                    1.0f, goMax);
          }
        }
        drawEditXgoto();
      }
      if (acceptPressed) {
        drawStatus("Xgoto set");
        const double signedEx = _xGotoDirCw ? double(_xGotoValue)
                                            : -double(_xGotoValue);
        _stretcher.gotoExpansion(signedEx);
        _statusUntilMs = millis() + 800;
        _ui = UiState::RUNNING;
      }
      if (menuPressed) {
        _ui = UiState::MENU;
        drawMenu();
      }
    } break;

    case UiState::EXPERIMENTS_MENU: {
      if (!_runner || _runner->experimentCount() == 0) {
        _ui = UiState::MENU;
        drawMenu();
        break;
      }
      const uint8_t n = _runner->experimentCount();
      if (ed == +1) {
        _expIndex = (_expIndex + 1) % n;
        drawExperimentsMenu();
      } else if (ed == -1) {
        _expIndex = (_expIndex + n - 1) % n;
        drawExperimentsMenu();
      }
      if (downPressed) {
        _expIndex = (_expIndex + 1) % n;
        drawExperimentsMenu();
      }
      if (acceptPressed) {
        const IrisExperiment* exp = _runner->experiment(_expIndex);
        if (exp) {
          _runner->requestRun(*exp);
          _ui = UiState::RUNNING_EXPERIMENT;
          _lastRunStatusMs = 0;
          drawRunningExperiment();
        }
      }
      if (menuPressed) {
        _ui = UiState::MENU;
        drawMenu();
      }
    } break;

    case UiState::RUNNING_EXPERIMENT: {
      // Refresh the LCD a few times per second to reflect runner state.
      const uint32_t now = millis();
      if (now - _lastRunStatusMs >= 250) {
        drawRunningExperiment();
        _lastRunStatusMs = now;
      }
      // MENU = abort. The runner will honour it between motion segments.
      if (menuPressed && _runner) _runner->requestAbort();
      // When the runner returns to idle, pop back to the experiments submenu.
      if (_runner && !_runner->isRunning()) {
        _ui = UiState::EXPERIMENTS_MENU;
        drawExperimentsMenu();
      }
    } break;
  }
}

} // namespace iris
} // namespace kisley
