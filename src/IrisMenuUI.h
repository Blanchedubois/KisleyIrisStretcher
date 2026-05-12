#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "IrisStretcher.h"
#include "internal/DebouncedButton.h"
#include "internal/QuadEncoder.h"

namespace kisley {
namespace iris {

// "About" screen metadata. Defaults come from library build; override with
// setAboutInfo() in your sketch to display the sketch's build info instead.
struct IrisAboutInfo {
  const char* fwName    = "Iris Stretcher";
  const char* fwVersion = "v1.0.0";
  const char* labLine1  = "Kisley Lab";
  const char* labLine2  = "CWRU";
  const char* creator   = "Tejasvin Shrikanth";
  const char* buildDate = __DATE__;
  const char* buildTime = __TIME__;
};

class IrisMenuUI {
public:
  struct Pins {
    uint8_t btnMenu;
    uint8_t btnDown;
    uint8_t btnAccept;
    uint8_t encA;
    uint8_t encB;
    uint8_t encSW;
    uint8_t sda = 3;
    uint8_t scl = 4;
  };

  using ActionCallback = void(*)(IrisMenuUI& ui, void* user);

  IrisMenuUI(IrisStretcher& stretcher, const Pins& pins, Stream& io = Serial);

  void begin();    // initializes I2C, LCD, buttons, encoder, splash screen
  void update();   // call from loop() — fully non-blocking

  void setInvertEncoder(bool on)         { _invertEncoder = on; }
  void setUseInternalPulldown(bool on)   { _useInternalPulldown = on; }
  void setAboutInfo(const IrisAboutInfo& info) { _about = info; }
  void setShowSplash(bool on)            { _showSplash = on; }
  void setDetectedLcdAddress(uint8_t a)  { _lcdAddress = a; }

  uint8_t lcdAddress() const             { return _lcdAddress; }

  // Append a custom menu item. Returns false if MAX_MENU_ITEMS reached.
  bool registerMenuItem(const char* label,
                        ActionCallback cb,
                        void* user = nullptr);

  // Public for advanced sketches that want direct LCD access.
  hd44780_I2Cexp& lcd() { return _lcd; }

  // Editable parameter ranges and step sizes — labs can tune these.
  float xSpeedMin       = 0.1f;
  float xSpeedMax       = 5.0f;
  float xSpeedStepFine  = 0.01f;
  float xSpeedStepCoarse = 0.10f;
  float xGotoStepFine   = 0.001f;
  float xGotoStepCoarse = 0.050f;

private:
  // ---- State ----
  enum class UiState : uint8_t {
    MENU,
    RUNNING,
    HELP_SCROLL,
    ABOUT_SCROLL,
    EDIT_XSPEED,
    EDIT_XGOTO
  };

  enum class BuiltinKind : uint8_t {
    XsetZero, Xzero, Xcalibrate, Xspeed, Xgoto, Xhelp, Xabout, Custom
  };

  struct MenuEntry {
    const char*    label;
    BuiltinKind    kind;
    ActionCallback callback;
    void*          user;
  };

  static constexpr uint8_t MAX_MENU_ITEMS = 16;

  // ---- Helpers ----
  uint8_t scanI2CForLCD();
  void drawMenu();
  void drawStatus(const char* msg);
  void drawEditXspeed();
  void drawEditXgoto();
  void printFloatFixed(float v, uint8_t places);
  void runCurrentItem();
  void enterHelp();
  void enterAbout();
  void tickHelp();
  void tickAbout();

  // ---- Refs ----
  IrisStretcher&  _stretcher;
  Stream&         _io;
  Pins            _pins;

  // ---- Devices ----
  hd44780_I2Cexp  _lcd;
  uint8_t         _lcdAddress = 0x00;

  DebouncedButton _btnMenu;
  DebouncedButton _btnDown;
  DebouncedButton _btnAccept;
  DebouncedButton _btnEncSW;
  QuadEncoder     _enc;

  // ---- Menu ----
  MenuEntry _items[MAX_MENU_ITEMS];
  uint8_t   _itemCount = 0;
  uint8_t   _menuIndex = 0;

  // ---- UI state ----
  UiState        _ui = UiState::MENU;
  unsigned long  _statusUntilMs = 0;
  bool           _invertEncoder = false;
  bool           _useInternalPulldown = true;
  bool           _showSplash = true;

  // ---- Edit values ----
  float _xSpeedValue = 1.0f;
  float _xGotoValue  = 1.000f;   // magnitude, range [1.0, maxEx]
  bool  _xSpeedFineMode = false;
  bool  _xGotoFineMode  = false;
  bool  _xGotoDirCw     = true;  // CW default; toggled by DOWN button in edit screen

  // ---- Help/About paging ----
  IrisAboutInfo  _about;
  uint8_t        _scrollFrame = 0;
  unsigned long  _scrollNextMs = 0;
};

} // namespace iris
} // namespace kisley
