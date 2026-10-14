# Subsystem 5 — Adaptive ultrasonic scanning and obstacle profiling

HC-SR04 ultrasonic sensor + SG90 servo, on the Raspberry Pi Pico W, running on
micro T-Kernel 3.0 (SMP). Student ID: `<student ID>`.

This folder holds everything for Buddy 5. No other subsystem needs to touch
these files. Other subsystems only use the event flag in `subsystem5.h`.

---

## 1. Wiring (read this before powering anything)

### 1.1 Pin summary

| Part | Part pin | Pico W GPIO | Pico W physical pin | Notes |
|---|---|---|---|---|
| HC-SR04 | VCC | – | **40 (VBUS, 5 V)** | The module needs 5 V. At 3.3 V it is unreliable. |
| HC-SR04 | TRIG | **GP2** | **4** | Pico output (3.3 V is enough to trigger it). |
| HC-SR04 | ECHO | **GP3** | **5** | **Output is 5 V: use the divider in 1.2. Never connect it straight to GP3.** |
| HC-SR04 | GND | – | **3 (GND)** | Shared ground. |
| SG90 servo | Signal (orange or yellow) | **GP15** | **20** | 50 Hz PWM, PWM slice 7, channel B. |
| SG90 servo | V+ (red) | – | **40 (VBUS, 5 V)** or separate 5 V supply | See 1.3. |
| SG90 servo | GND (brown) | – | **18 (GND)** | Shared ground. |

Other pins near these (GP0, GP1, GP4 to GP14) are **not used** by Buddy 5.
GP23, GP24, GP25, GP29 are the Pico W radio, so leave them alone.
The same claims are listed in `TEAM.md` (pin table).

### 1.2 ECHO voltage divider (5 V → 3.3 V)

The RP2040 pins are **not 5 V tolerant**. Build this between the sensor and the Pico:

```
HC-SR04 ECHO ──[ 1 kΩ ]──┬────────────► Pico GP3 (pin 5)
                         │
                       [ 2 kΩ ]
                         │
                        GND
```

Output = 5 V × 2 / (1 + 2) = **3.3 V**. If you have no 2 kΩ resistor, use two
1 kΩ resistors in series for the bottom leg. Any pair with a 1 : 2 ratio works
(for example 10 kΩ and 20 kΩ).

### 1.3 Servo power

- An SG90 can pull several hundred mA when it starts moving or is blocked. A
  laptop USB port can sag, which resets the Pico (the serial port disappears and
  reappears).
- **Best:** power the servo from a separate 5 V supply (for example 4 × AA through a
  holder). Connect the **supply GND to Pico GND** (all grounds must be common).
- **Also add** a 100 to 470 µF capacitor across the servo's V+ and GND.
- **For a quick bench test only:** VBUS (pin 40) works if the Pico resets are not a
  problem. Do not draw the servo from the 3V3 pin.

### 1.4 Full picture

```
                       Pico W (top view, USB at the top)
                  ┌──────────────────────────────┐
   (unused) GP0 ─ 1                              40 ─ VBUS (5 V) ──┬── HC-SR04 VCC
   (unused) GP1 ─ 2                              39                │   (and SG90 red,
        GND ─────  3 ──────────────────┐         38 ─ GND          │    bench test only)
   TRIG ◄── GP2 ─ 4                    │         ...               │
   ECHO ──► GP3 ─ 5  (via divider)     │         ...
                  ...                  │
        GND ───── 18 ─────────────┐    │
   (unused) GP14 ─ 19             │    │
  Servo signal ◄── GP15 ─ 20      │    │
                  └───────────────┼────┼─────────┘
                                  │    └── HC-SR04 GND
                                  └─────── SG90 GND (brown)
```

Pin positions follow the standard Pico pinout. Check against the printed labels
on your board, because the numbering runs down the left side from 1 to 20, then
up the right side from 21 to 40.

### 1.5 Sensor mounting

- Fix the HC-SR04 to the servo horn so the sensor turns with it. At servo angle 0°
  it faces **straight ahead**.
- **+90° = left, -90° = right** (see `us_config.h`). If your sensor looks the wrong
  way, swap the signs of `SERVO_ANGLE_LEFT` and `SERVO_ANGLE_RIGHT`, or re-seat the
  horn.
- Keep wires clear of the sensor's cone and keep it at least 2 cm from walls.

---

## 2. What the code does

State flow (from the Week 6 statechart):

```
INIT → FRONT_MONITOR → AVOID_STANDBY → OBSTACLE_PROFILING → (back to AVOID_STANDBY)
        ping every 20 ms     latched,          SCAN_LEFT +90°  → SCAN_RIGHT -90°
        median of 3           waiting for       → SELECT_SIDE → RECENTRE (0°)
        2 hits ≤ 150 mm       Nav
        → obstacle flag
```

Not yet implemented: CLEARANCE_CHECK, SENSOR_FAULT / FAULT_STOP, coarse + fine
scan and obstacle width estimate.

### Files

| File | Role |
|---|---|
| `subsystem5.h / .c` | **Public interface** and the `s5_obstacle` task (the state flow above). |
| `us_scan.h / .c` | Logic: filtering, `us_distance_mm`, `us_object_detected`, `us_scan_left_right`, side selection. No hardware access. |
| `drv_ultrasonic.h / .c` | Hardware layer: TRIG pulse, ECHO edge interrupt, timestamps, servo PWM. |
| `us_config.h` | **All tunables in one place** (pins, thresholds, servo limits, timings). |
| `s5_demo.c` | Bench stand-in for Navigation. Set `US_DEMO` to 0 in `us_config.h` when the real Nav code takes over. |
| `subsystem5_test.c` | Automatic logic tests S5-U01 to U09. |

### Functions other code can call

| Function | Returns |
|---|---|
| `bool us_object_detected(int32_t threshold_mm, int32_t *dist_mm)` | `true` if something is closer than the threshold. `dist_mm` receives the distance (may be NULL). |
| `int32_t us_distance_mm(void)` | Filtered distance in mm, or `US_NO_ECHO` (-1, nothing in range) / `US_INVALID` (-2, bad reading). |
| `void us_scan_left_right(us_profile_t *out)` | Sweeps left, then right, picks the clearer side, recentres (about 1 s). |

Normally Nav should not call these directly. Use the event flag below, so only one
task touches the sensor and servo.

---

## 3. Interface for Navigation (IF-06): event flag `flg_obstacle`

Include `subsystem5.h`. The flag is created in `subsystem5_init()`.

| Bit | Name | Direction | Meaning |
|---|---|---|---|
| 0x01 | `S5_FLG_CALIBRATED` | Nav → S5 | Nav finished calibration, S5 may start monitoring. |
| 0x02 | `S5_FLG_FRONT_OBSTACLE` | S5 → Nav | Obstacle at or under 15 cm (2 filtered readings in a row). **Latched** until `AVOID_DONE`. |
| 0x04 | `S5_FLG_PROFILE_REQ` | Nav → S5 | Nav asks for a left/right profile. |
| 0x08 | `S5_FLG_PROFILE_DONE` | S5 → Nav | `s5_profile` is valid. |
| 0x10 | `S5_FLG_AVOID_DONE` | Nav → S5 | Nav left OBSTACLE_AVOIDANCE, S5 returns to FRONT_MONITOR. |

Result (read after `PROFILE_DONE`): `s5_profile.clear_L_mm`, `s5_profile.clear_R_mm`,
`s5_profile.side` (`AVOID_LEFT` or `AVOID_RIGHT`; a tie within 3 cm gives LEFT).
`s5_last_mm` holds the latest filtered distance for telemetry.

This interface is **proposed**. Nav owner, please confirm or change the bits.

Typical Nav sequence:

```c
tk_set_flg(flg_obstacle, S5_FLG_CALIBRATED);                       // once
tk_wai_flg(flg_obstacle, S5_FLG_FRONT_OBSTACLE, TWF_ORW, &ptn, TMO_FEVR);
tk_set_flg(flg_obstacle, S5_FLG_PROFILE_REQ);                      // stop and ask
tk_wai_flg(flg_obstacle, S5_FLG_PROFILE_DONE, TWF_ORW | TWF_CLR, &ptn, 5000);
// ... turn toward s5_profile.side ...
tk_set_flg(flg_obstacle, S5_FLG_AVOID_DONE);                       // hand back to S5
```

---

## 4. Kernel resources used (for the shared budget in `TEAM.md`)

| Resource | Use |
|---|---|
| Tasks | `s5_obstacle` (priority **20**, core 0) and `s5_demo` (priority **21**, core 0, bench only) |
| Event flags | `flg_obstacle` (IF-06) and one internal flag for the ECHO interrupt |
| Interrupt | GPIO edge interrupt on `IO_BANK0` (vector 13), handler registered by `drv_ultrasonic.c`, interrupt priority 2 |
| PWM | slice 7 (GP15), clock divider set so 1 tick = 1 µs, wrap 19999 (20 ms period) |
| Timer | Reads the RP2040 `TIMER_TIMERAWL` register (1 MHz) for echo timestamps. Read-only. |

**Shared-hardware warning:** there is only one `IO_BANK0` interrupt vector, and
`tk_def_int` allows one handler for it. If another subsystem also wants GPIO
interrupts, it must be coordinated with Buddy 5 (a single shared handler), or the
later registration will replace the earlier one.

---

## 5. Build, flash and see the output

```bash
export PICO_SDK_PATH=$HOME/pico-sdk        # needed for the default USB + Wi-Fi build
cd build_make
make clean
make -j8
```

The result is `mtk3pico_smp1_usb_cdc_wifi.uf2` in `build_make/`. Hold BOOTSEL while
plugging in the Pico, then drag it onto the `RPI-RP2` drive. The serial console
appears as a COM port (115200 baud).

Expected output, with the sensor wired:

```
[s5] started: threshold=150 mm
[s5] dist=312 mm
[s5] OBSTACLE at 137 mm -> profiling
[s5] clear_L=420 mm  clear_R=95 mm  -> avoid LEFT
```

`dist=clear` means no echo came back within 18 ms (open space, or nothing
connected). With the sensor unplugged you will see `clear` forever, so it does not
by itself prove the wiring is right.

Do not commit `build_make/` build outputs.

---

## 6. Tests

Cases and results are in `testing/subsystem5/TEST_LOG.md`.
S5-U01 to U09 run automatically (`make TEST=1`). S5-U10 to U15 are manual bench
tests: distance accuracy (±2 cm), obstacle flag at 15 cm, latency (≤ 50 ms),
left/right choice, scan time (about 1 s) and open-space reads clear.

**Status:** the firmware builds and boots on a Pico W with the `[s5]` task running.
The bench tests with the real sensor and servo have **not** been run yet.

## 7. Known limits and open items

- Obstacle latency of 50 ms (FR-PER-16) is not yet measured (test S5-U12). The loop
  period is 20 ms plus the ping time, with two confirming readings.
- Servo end points (500 and 2400 µs) are typical SG90 values, so tune them on the
  bench in `us_config.h`.
- The 5 V ECHO divider and servo power are the two hardware items most likely to
  cause trouble.
