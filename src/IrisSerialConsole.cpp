#include "IrisSerialConsole.h"
#include "IrisExperiment.h"
#include <string.h>
#include <stdlib.h>

namespace kisley {
namespace iris {

namespace {
// Case-insensitive equality for short ASCII tokens. Arduino doesn't
// portably provide strcasecmp, so we hand-roll one.
bool ieq(const char* a, const char* b) {
  while (*a && *b) {
    char ca = (*a >= 'A' && *a <= 'Z') ? char(*a + 32) : *a;
    char cb = (*b >= 'A' && *b <= 'Z') ? char(*b + 32) : *b;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == *b;
}
} // namespace

IrisSerialConsole::IrisSerialConsole(IrisStretcher& stretcher, Stream& io)
  : _stretcher(stretcher), _io(io) {}

void IrisSerialConsole::begin() {
  printBanner();
  printHelp();
}

void IrisSerialConsole::printBanner() {
  _io.println(" ___      _       ____  _            _       _               ");
  _io.println("|_ _|_ __(_)___  / ___|| |_ _ __ ___| |_ ___| |__   ___ _ __ ");
  _io.println(" | || '__| / __| \\___ \\| __| '__/ _ \\ __/ __| '_ \\ / _ \\ '__|");
  _io.println(" | || |  | \\__ \\  ___) | |_| | |  __/ || (__| | | |  __/ |   ");
  _io.println("|___|_|  |_|___/ |____/ \\__|_|  \\___|\\__\\___|_| |_|\\___|_|   ");
  _io.println("+===============+");
  _io.println(_bannerLine);
  _io.println("+===============+");
}

void IrisSerialConsole::printHelp() {
  _io.println("Commands:");
  _io.println("________");
  _io.println(" Xgoto <Ex> [cw|ccw]  \xe2\x80\x93 move to Ex magnitude (CW default)");
  _io.println(" Xzero                \xe2\x80\x93 drive motor to zero Position");
  _io.println(" XsetZero             \xe2\x80\x93 reset position counter to 0");
  _io.println(" Xcalibrate           \xe2\x80\x93 run calibration routine");
  _io.println(" Xspeed <value> cm/s  \xe2\x80\x93 changes expansion speed (cm/s approx)");
  _io.println(" Xstrain              \xe2\x80\x93 toggle 9-ADC strain CSV streaming");
  _io.println(" Xrun <name>          \xe2\x80\x93 run a registered experiment (Xrun list)");
  _io.println(" Xabort               \xe2\x80\x93 abort a running experiment");
  _io.println(" Xhelp                \xe2\x80\x93 this message");
  for (uint8_t i = 0; i < _customCount; i++) {
    if (_custom[i].help) {
      _io.print(" X");
      _io.print(_custom[i].name);
      _io.print("  \xe2\x80\x93 ");
      _io.println(_custom[i].help);
    }
  }
}

void IrisSerialConsole::attachRunner(IrisExperimentRunner& runner) {
  _runner = &runner;
}

bool IrisSerialConsole::registerCommand(const char* name,
                                        const char* helpLine,
                                        CommandCallback cb,
                                        void* user) {
  if (!name || !cb) return false;
  if (_customCount >= MAX_CUSTOM_CMDS) return false;
  _custom[_customCount++] = {name, helpLine, cb, user};
  return true;
}

void IrisSerialConsole::update() {
  while (_io.available() && !_ready) {
    char c = (char)_io.read();
    if (c == '\n' || c == '\r') {
      if (_idx > 0) {
        _buf[_idx] = '\0';
        _ready = true;
      }
    } else if (_idx < MAX_CMD_LEN - 1) {
      _buf[_idx++] = c;
    }
  }
  if (_ready) {
    parseCommand();
    _idx = 0;
    _ready = false;
  }
}

void IrisSerialConsole::parseCommand() {
  if (_buf[0] != 'X') {
    _io.println("Invalid prefix. Commands must start with 'X'.");
    return;
  }
  char* p   = _buf + 1;
  char* cmd = strtok(p, " ");
  if (!cmd) return;

  if (dispatchBuiltin(cmd, nullptr)) return;

  // Custom commands
  for (uint8_t i = 0; i < _customCount; i++) {
    if (strcmp(cmd, _custom[i].name) == 0) {
      char* rest = strtok(nullptr, "");        // remainder of line
      _custom[i].cb(rest ? rest : "", _custom[i].user);
      return;
    }
  }

  _io.print("Unknown command: ");
  _io.println(cmd);
}

bool IrisSerialConsole::dispatchBuiltin(const char* cmd, char* /*tokState*/) {
  if (strcmp(cmd, "goto") == 0) {
    char* arg = strtok(nullptr, " ");
    if (!arg) { _io.println("Usage: Xgoto <Ex> [cw|ccw]"); return true; }
    const double magnitude = atof(arg);

    // Optional direction token; defaults to CW.
    char* dirArg = strtok(nullptr, " ");
    bool cw = true;
    if (dirArg) {
      if      (ieq(dirArg, "ccw")) cw = false;
      else if (ieq(dirArg, "cw"))  cw = true;
      else {
        _io.print("Unknown direction: "); _io.println(dirArg);
        _io.println("Usage: Xgoto <Ex> [cw|ccw]");
        return true;
      }
    }

    _stretcher.gotoExpansion(cw ? magnitude : -magnitude);
    return true;
  }
  if (strcmp(cmd, "zero") == 0) {
    _stretcher.goToZero();
    return true;
  }
  if (strcmp(cmd, "setZero") == 0) {
    _stretcher.setZeroHere();
    return true;
  }
  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return true;
  }
  if (strcmp(cmd, "calibrate") == 0) {
    _stretcher.calibrate();
    return true;
  }
  if (strcmp(cmd, "strain") == 0) {
    if (!_runner) {
      _io.println(F("Error: no IrisExperimentRunner attached"));
      return true;
    }
    const bool wasOn = _runner->isStreamingStrain();
    _runner->setStreamStrain(!wasOn);
    _io.print(F("# Strain streaming: "));
    _io.println(_runner->isStreamingStrain() ? F("ON") : F("OFF"));
    return true;
  }
  if (strcmp(cmd, "run") == 0) {
    if (!_runner) {
      _io.println(F("Error: no IrisExperimentRunner attached"));
      return true;
    }
    char* arg = strtok(nullptr, " ");
    if (!arg) {
      _io.println(F("Usage: Xrun <name> | Xrun list"));
      return true;
    }
    if (ieq(arg, "list")) {
      _io.print(F("# Registered experiments ("));
      _io.print(_runner->experimentCount()); _io.println(F("):"));
      for (uint8_t i = 0; i < _runner->experimentCount(); i++) {
        const IrisExperiment* e = _runner->experiment(i);
        if (e) { _io.print(F("#   ")); _io.println(e->name); }
      }
      return true;
    }
    if (!_runner->requestRunByName(arg)) {
      _io.print(F("Error: no experiment named ")); _io.println(arg);
    }
    return true;
  }
  if (strcmp(cmd, "abort") == 0) {
    if (!_runner) {
      _io.println(F("Error: no IrisExperimentRunner attached"));
      return true;
    }
    _runner->requestAbort();
    _io.println(F("# Abort requested"));
    return true;
  }
  if (strcmp(cmd, "speed") == 0) {
    char* arg = strtok(nullptr, " ");
    if (!arg) { _io.println("Usage: Xspeed <bladeSpeed_cm/s>"); return true; }
    const double sb = atof(arg);
    if (sb <= 0.0) {
      _io.println("Error: speed must be > 0");
    } else {
      _io.print("Blade speed set to: ");
      _io.print(sb, 4);
      _io.println(" cm/s");
      _stretcher.setBladeSpeed(sb);
    }
    return true;
  }
  return false;
}

} // namespace iris
} // namespace kisley
