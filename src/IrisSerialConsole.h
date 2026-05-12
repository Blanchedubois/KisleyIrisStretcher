#pragma once
#include <Arduino.h>
#include "IrisStretcher.h"

namespace kisley {
namespace iris {

// Reads "X<command> [args]" lines from a Stream and dispatches them.
// Built-in commands:  Xgoto <Ex>, Xzero, XsetZero, Xcalibrate,
//                     Xspeed <cm/s>, Xhelp.
//
// Labs can append their own commands via registerCommand().
class IrisSerialConsole {
public:
  using CommandCallback = void(*)(const char* args, void* user);

  IrisSerialConsole(IrisStretcher& stretcher, Stream& io = Serial);

  // Prints banner + help. Does NOT start the underlying Stream — the sketch
  // is responsible for Serial.begin() / USB.begin() before this is called.
  void begin();

  // Call from loop(). Non-blocking.
  void update();

  // Append a custom command. `name` is the bare command (no "X" prefix);
  // sending "Xfoo bar baz" with name="foo" calls cb with args="bar baz".
  bool registerCommand(const char* name,
                       const char* helpLine,
                       CommandCallback cb,
                       void* user = nullptr);

  void printBanner();
  void printHelp();

  // Override the single mid-banner identification line (default "KisleyLab V1.0").
  void setBannerLine(const char* line) { _bannerLine = line; }

private:
  void parseCommand();
  bool dispatchBuiltin(const char* cmd, char* tokenizerState);

  static constexpr uint8_t MAX_CMD_LEN     = 64;
  static constexpr uint8_t MAX_CUSTOM_CMDS = 8;

  struct CustomCmd {
    const char*     name;
    const char*     help;
    CommandCallback cb;
    void*           user;
  };

  IrisStretcher& _stretcher;
  Stream&        _io;

  char     _buf[MAX_CMD_LEN];
  uint8_t  _idx = 0;
  bool     _ready = false;

  CustomCmd _custom[MAX_CUSTOM_CMDS];
  uint8_t   _customCount = 0;

  const char* _bannerLine = "|KisleyLab V1.0 |";
};

} // namespace iris
} // namespace kisley
