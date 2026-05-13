# KisleyIrisStretcher

Arduino library for the **Kisley Lab iris stretcher** (Case Western
Reserve University) — a stepper-driven mechanical iris that radially
stretches a sample to a programmable expansion ratio.

This library is the modular refactor of `IrisModule1.0.ino`. It exposes
a small high-level API so other labs can drop in a single 15-line sketch
and either run the rig as-is or build their own experiments on top.

---

## Install

**Arduino IDE → Sketch → Include Library → Add .ZIP Library…**, point at
this folder zipped, or symlink it into `~/Documents/Arduino/libraries/`.

Dependencies (auto-installed by Library Manager, or install yourself):

- `AccelStepper`
- `hd44780` (Bill Perry)
- `Adafruit NAU7802` *(only if you use the strain ADC)*

Target board: **ESP32-S3** (Arduino-ESP32 core). The library is single-
architecture by design — `library.properties` declares `architectures=esp32`.

## Quick start (parity with the original IrisModule1.0)

```cpp
#include <USB.h>
#include <KisleyIrisStretcher.h>
using namespace kisley::iris;

IrisStretcher stretcher(/*STEP=*/12, /*DIR=*/13);
IrisMenuUI    ui(stretcher, IrisMenuUI::Pins{A0, A1, A2, A5, A4, A3, 3, 4});
IrisSerialConsole console(stretcher);

void setup() {
  USB.begin();
  Serial.begin(115200);
  while (!Serial) {}
  stretcher.begin();
  ui.begin();
  console.begin();
}

void loop() {
  ui.update();
  console.update();
}
```

The `examples/IrisModule1.0/` sketch is a fully-annotated version of
the above and reproduces the original firmware behavior 1:1 — same
splash screen, menu items, encoder feel, serial banner, and command
responses.

## API at a glance

| Class | Purpose | Key methods |
|---|---|---|
| `IrisGeometry` | Mechanical constants (`r0`, `rp`, `Y0`, gear ratio, …) | direct field access |
| `IrisStretcher` | High-level motion + kinematics facade | `gotoExpansion`, `goToZero`, `setZeroHere`, `calibrate`, `setBladeSpeed`, `currentSteps`, `currentTheta` |
| `IrisMenuUI` | LCD + encoder + buttons state machine | `begin`, `update`, `registerMenuItem`, `setInvertEncoder`, `setUseInternalPulldown`, `setAboutInfo` |
| `IrisSerialConsole` | `X<command>` line parser | `begin`, `update`, `registerCommand`, `printBanner`, `printHelp` |
| `IrisStrainNAU7802` | Optional 24-bit single-chip strain gauge ADC | `begin`, `readVolts` |
| `IrisStrainArray` | Multi-mux strain-gauge array (9 NAU7802s on the Kisley rig) | `begin`, `acquireRow`, `tare`, `printCsvRow` |
| `IrisExperiment` / `IrisExperimentRunner` | Define ordered expansion sequences and stream synchronised CSV data | `registerExperiment`, `requestRun`, `update` |
| `IrisKinematics` | Pure-math forward/inverse maps (static) | `computeEx`, `findTheta` |

### Serial commands

| Command | Effect |
|---|---|
| `Xgoto <Ex> [cw\|ccw]` | Move to magnitude `Ex` in `[1.0, maxEx]`. Direction defaults to `cw`; `ccw` rotates the motor the same θ magnitude in the opposite direction. `Ex` at or below 1.0 returns to center (θ=0). |
| `Xzero` | Drive the motor back to θ = 0 |
| `XsetZero` | Reset the position counter to 0 at the current pose |
| `Xcalibrate` | Run the calibration routine |
| `Xspeed <cm/s>` | Set blade speed (recomputes step delay) |
| `Xstrain` | Toggle continuous 9-ADC CSV streaming on/off |
| `Xrun <name>` | Run a registered experiment by name (`Xrun list` to enumerate) |
| `Xabort` | Abort the currently running experiment (between motion segments) |
| `Xhelp` | Print this list |

### Defining and running experiments

```cpp
#include <KisleyIrisStretcher.h>
using namespace kisley::iris;

IrisStretcher        stretcher(12, 13);
IrisStrainArray      strain;
IrisMenuUI           ui(stretcher, /* pins */);
IrisSerialConsole    console(stretcher);
IrisExperimentRunner runner(stretcher, strain);

// 1. Define waypoints (signed Ex; positive=CW, negative=CCW, |Ex|≤1=center).
const float kExp1Targets[] = { 1.0f, 3.4f, 1.0f };
const IrisExperiment kExp1 = { "Exp1", kExp1Targets, 3 };

void setup() {
  /* ... usual init ... */
  strain.begin();
  runner.registerExperiment(kExp1);
  ui.attachRunner(runner);
  console.attachRunner(runner);
}

void loop() {
  ui.update();
  console.update();
  runner.update();   // <- drives the experiment state machine
}
```

The LCD's main menu gains an "Experiments" entry; selecting it pushes
into a submenu listing every registered experiment. ACCEPT runs;
MENU aborts. CSV output per row:
`exp,t_ms,steps,target_ex,state,ADC1_mean,ADC1_std,…,ADC9_mean,ADC9_std`.

See `examples/ExperimentDemo/` for stepped, bidirectional, and
`customRun`-based examples.

**On the LCD**, the Xgoto edit screen shows magnitude + direction:
`"1.350 CW  [SW]"` or `"1.350 CCW [SW]"`. The encoder edits magnitude
(range `[1.0, maxEx]`); the **DOWN** button toggles CW ↔ CCW; **SW**
toggles fine/coarse; **ACCEPT** commits.

Step count is signed and absolute, so a round-trip
`1.3 CW → 1.0 → 1.3 CW` lands on byte-identical step positions, and
`1.3 CW → 1.3 CCW` lands on exactly mirrored steps. See
`BIDIRECTIONAL_XGOTO.md` for the kinematic rationale.

**API equivalent** for programmatic use:
```cpp
stretcher.gotoExpansion( 1.35);  // CW
stretcher.gotoExpansion(-1.35);  // CCW (same θ magnitude, opposite direction)
stretcher.gotoExpansion( 1.0);   // center
```

## Wiring

| Function | ESP32-S3 pin |
|---|---|
| STEP | GPIO 12 |
| DIR | GPIO 13 |
| I2C SDA | GPIO 3 |
| I2C SCL | GPIO 4 |
| MENU button | A0 |
| DOWN button | A1 |
| ACCEPT button | A2 |
| Encoder switch | A3 |
| Encoder B | A4 |
| Encoder A | A5 |

Buttons are wired with `INPUT_PULLDOWN` (active HIGH). Set
`ui.setUseInternalPulldown(false)` if your board uses external pulldowns.

## Extending the library

Other labs building custom experiments on the rig can add behavior
without forking library source:

```cpp
void runMyExperiment(IrisMenuUI&, void*) {
  stretcher.gotoExpansion(1.5);
  delay(2000);
  stretcher.gotoExpansion(2.0);
}

void cmdSweep(const char* args, void* user) {
  for (float ex = 1.1; ex <= 3.0; ex += 0.1) {
    stretcher.gotoExpansion(ex);
  }
}

void setup() {
  /* ... usual init ... */
  ui.registerMenuItem("MyExp", runMyExperiment);
  console.registerCommand("sweep", "Run 1.1→3.0 sweep", cmdSweep);
}
```

A higher-level "Experiments" subsystem built on these hooks is on the
roadmap.

## Differences from monolithic `IrisModule1.0.ino`

These are deliberate corrections vs. the original 794-line sketch:

1. **Encoder switch press in Xspeed/Xgoto edit screens now toggles
   fine/coarse only.** The original code intended SW to *also* commit, but
   a double-`fell()` consumption bug meant only ACCEPT could commit.
   Observable behavior unchanged; dead branch removed.
2. **Help and About scroll screens are non-blocking.** The original used
   `delay()` totaling ~7 s. The new version uses millis-based paging so
   the encoder and serial reader stay responsive, and pressing ACCEPT or
   MENU during scrolling returns to the main menu immediately.
3. **Kinematics promoted to `double` end-to-end.** The original mixed
   `float` arithmetic with `1e-10` Newton-Raphson tolerances that fell
   below float epsilon (~1.2e-7). Free on ESP32-S3 FPU.
4. **Verbose debug prints removed from inside the math.** The original
   called `Serial.print` on every kinematics evaluation, which fired
   hundreds of times per `Xgoto`.
5. **Lab name spelling.** The original firmware printed "Kisely Lab" /
   "KiselyLab"; corrected to **Kisley** throughout.

The default example sketch sets `console.setBannerLine("|KisleyLab V2.1 |")`
to match the original `verBuf=2.1` runtime banner.

## License

MIT — see `LICENSE`.
