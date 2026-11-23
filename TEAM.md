# Team and subsystems

The robot car: IR line following, IMU (tilt, heading, humps, lift/overturn
safety stop), Code 39 barcodes, an ultrasonic stop before obstacles and
WiFi/MQTT telemetry, on micro T-Kernel 3.0 on the Pico W. Fill in the Student ID and GitHub columns.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).
Each subsystem is written as a **state machine** (named in section 1).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | State machine | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Comms: WiFi/MQTT telemetry, START/STOP/RESET from the broker, status line | COM: DISCONNECTED / SYNC / CONNECTED (`com_state.c`) | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion: motors (PWM), wheel encoders (PIO), speed sync, emergency stop | STARTING / STOPPED / DRIVE / WHEELS / ESTOP (`task_motion.c`) | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Navigation: IR line following, buttons, run control (steers gently while a barcode goes past, IF-14) | IDLE / CALIBRATING / FOLLOWING / AVOIDING, and FOLLOW / CONFIRM / CROSSING (`task_nav.c`) | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU: tilt, heading, humps, lift/overturn safety stop | SEARCH / RUN / CAL_SPIN / CAL_SETTLE / CAL_ZERO (`task_imu.c`) | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Barcode: Code 39 on the right-most MH-sensor (LEFT / RIGHT / FORWARD) | WAIT_BAR / IN_PASS (`barcode.c`) | | | `app_program/subsystem5/` | `testing/subsystem5/` |
| 6 | Obstacle: HC-SR04 distance ahead on an SG90 servo; stops the car at 15 cm and measures the object | MONITOR / BLOCKED / FAULT / FAIL (`task_obstacle.c`) | | | `app_program/subsystem6/` | `testing/subsystem6/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S6 | HC-SR04 TRIG (Grove 1). **Clashes with `CRASH_REPORT=1` / `CONSOLE=uart`, which take GP0/GP1 for UART0: `subsystem6.h` refuses to compile in those builds.** |
| GP1 | S6 | HC-SR04 ECHO (Grove 1, GPIO edge interrupt) -- same clash as GP0 |
| GP2 | S4 | I2C1 SDA → GY-511 (LSM303D) |
| GP3 | S4 | I2C1 SCL → GY-511 (LSM303D) |
| GP4 | | |
| GP5 | | |
| GP6 | S2 | right wheel encoder B (PIO) |
| GP7 | | |
| GP8 | S2 | right motor A (PWM 4A) |
| GP9 | S2 | right motor B (PWM 4B) |
| GP10 | S2 | left motor B (PWM 5A) |
| GP11 | S2 | left motor A (PWM 5B) |
| GP12 | | |
| GP13 | S3 | left IR line sensor DO (MH-sensor #1) |
| GP14 | S3 | right IR line sensor DO (MH-sensor #2) |
| GP15 | S6 | SG90 servo that aims the HC-SR04 (PWM 7B, 50 Hz) |
| GP16 | S2 | left wheel encoder A (PIO) |
| GP17 | S2 | left wheel encoder B (PIO) |
| GP18 | S1 | error indicator: the Robo Pico's 2 WS2812 RGB LEDs (PIO) |
| GP19 | S1 | COM status LED (LED + 330 Ω to GND) |
| GP20 | S3 | START/STOP button (to GND, internal pull-up) |
| GP21 | S3 | IMU calibration button (to GND, internal pull-up) |
| GP22 | S1 | buzzer: the Robo Pico's on-board piezo (PWM 3A, 2.7 kHz) |
| GP26 | S2 | right wheel encoder A (PIO; digital, not ADC) |
| GP27 | | (ADC1 capable) |
| GP28 | S5 | barcode MH-sensor #3 AO (ADC2) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** Every other task asks the owning task through a kernel object
(section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | S4 | `imu_task` (own driver `drv_i2c.c`; the kernel I2C driver stays off: it would use GP6/GP7) | GP2, GP3 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S5 | `barcode_task` + its timer interrupt (own driver `drv_bc_adc.c`). **Nobody opens the kernel's ADC device "adca".** | GP28 |
| Timer alarm 3 (TIMER_IRQ_3) | S5 | `barcode_task` (2 kHz sampler interrupt, core 1) | — |
| PWM slice 3 | S1 | `telemetry_task` (`drv_buzzer.c`, the buzzer). GP6 shares this slice and channel but S2 drives it as PIO, so the tone never reaches that pin. | GP22 |
| PWM slice 7 | S6 | `obstacle_task` (`drv_ultrasonic.c`, the servo). Both it and `drv_motor.c` release the PWM block from reset, because the two tasks start at the same time. | GP15 |
| GPIO interrupt (IO_IRQ_BANK0) | S6 | `obstacle_task` (ECHO edges on GP1, priority 2, core 1). **Nobody else may call `tk_def_int(13, ...)`:** one handler serves the whole GPIO bank. The WiFi port never uses it (`cyw43_post_poll_hook` is empty). | GP1 |
| PWM slices 4, 5 | S2 | `motion_task` (`drv_motor.c`) | GP8–GP11 |
| PIO block not used by the radio (2 state machines) | S2 | `motion_task` (`drv_encoder.c`) | GP6, GP16, GP17, GP26 |
| GPIO inputs | S3 | `nav_task` (`drv_line.c`) | GP13, GP14, GP20, GP21 |
| GPIO in / out | S6 | `obstacle_task` (`drv_ultrasonic.c`) | GP0 (TRIG out), GP1 (ECHO in) |
| GPIO output | S1 | `telemetry_task` | GP19 |
| PIO: 1 state machine + 4 instruction slots, in any block with room (loaded after S2's encoders) | S1 | `telemetry_task` (`drv_ws2812.c`, RGB error LEDs) | GP18 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in), which also runs S1's `comms_task()` | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

How the car does it (section 7, change 5): the WiFi task calls subsystem 1's
`comms_task()` on every poll, in place of the template's demo client
(`mqtt_utk.c`, now unused). All lwIP/MQTT calls happen inside it, on the
WiFi task. `telemetry_task` only hands frames, barcode texts and commands
to and from it through comms.c's single-writer rings (IF-10).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4).
The car's bands follow how urgent each job is (agreed for the car):
motors first, then the safety check, then line following.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 2 Motion | 10–11 | motion_task – 10 – core 1 |
| 4 IMU | 12–13 | imu_task – 12 – core 1 |
| 3 Navigation | 14–15 | nav_task – 14 – core 1 |
| 5 Barcode | 16–17 | barcode_task – 16 – core 1 (+ timer alarm 3 interrupt, priority 2, core 1) |
| 1 Comms | 18–19 | telemetry_task – 18 – core 0 |
| 6 Obstacle | 20–21 | obstacle_task – 20 – core 1 (+ the ECHO GPIO interrupt, priority 2, core 1) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".
Core 0 also runs the kernel's WiFi task every 2 ms and the USB console, so
the tasks that steer the car are all on core 1 (`APP_CORE_CONTROL` in app.h).
Subsystem 6 sits below comms on purpose: it pings every 20 ms and sleeps in
between, and it is the ECHO *interrupt* (priority 2) that has to be prompt,
not the task.

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S3 → S2 | message buffer (owned by S2) | `mbf_motion` | `MOTION_CMD` (`subsystem2.h`): STOP, DRIVE (synced %), WHEELS (direct %), ENABLE (new run: clear ESTOP, retry failed encoders) | |
| IF-02 | S4 → S2 | message buffer (owned by S2) | `mbf_motion` | `MOTION_CMD_ESTOP`: brake now, ignore DRIVE/WHEELS until ENABLE | |
| IF-03 | S2 → S1, S3, S4 | shared data + mutex (owned by S2) | `mtx_odom`, `motion_get_odometry()` | `ODOMETRY`: encoder counts, speed, motion state | |
| IF-04 | S3 → S4 | message buffer (owned by S4) | `mbf_imu_req` | `IMU_REQ`: CAL_START, CAL_ABORT, RESET_LIFT, RESET_RUN, CLEAR_ERRORS (every GP20 / START / RESET) | |
| IF-05 | S4 → S3 | event flag (owned by S4) | `flg_imu` | OVERTURNED, LIFTED, CAL_SPIN_DONE, CAL_DONE, CAL_FAILED (S4 sets, S3 clears) | |
| IF-06 | S4 → S1, S3 | shared data + mutex (owned by S4) | `mtx_imu`, `imu_get_status()`, `imu_take_hump()` | `IMU_STATUS`: angles, heading, humps, lift, state | |
| IF-07 | S1 → S3 | message buffer (owned by S3) | `mbf_nav_cmd` | `NAV_CMD`: START, STOP, RESET (from the broker) | |
| IF-08 | S3 → S1 | shared data + mutex (owned by S3) | `mtx_nav`, `nav_get_status()` | `NAV_STATUS`: run state, action, line sensors, junctions, run_count | |
| IF-09 | S5 → S1 | message buffer (owned by S1) | `mbf_barcode` | `BARCODE_MSG` (`subsystem1.h`): one per pass, valid + command (LEFT / RIGHT / FORWARD) + text. S1 beeps the command out on the buzzer | |
| IF-10 | S1 → kernel WiFi task | comms.c single-writer rings | `comms_send()`, `comms_send_barcode()`, `comms_send_obstacle()`, `comms_get_command()` | 10-byte frame (`data_layout.h`, version 2: it carries the measured object length in 6 bits, 63 = none yet), the barcode text and the obstacle scan line on their own topics, 1-byte command | |
| IF-11 | S6 → S3 | event flag (owned by S6) | `flg_obs` | OBSTACLE (latched: something within `S6_STOP_MM`), SCAN_DONE, SENSOR_FAULT, SENSOR_FAIL. **S6 sets AND clears these:** a latch is dropped only by its owner, on OBS_REQ_CLEAR | |
| IF-12 | S3 → S6 | message buffer (owned by S6) | `mbf_obs_req` | `OBS_REQ`: CLEAR (forget the obstacle and any fault; sent by every GP20 / START / RESET), SCAN (sweep the servo and measure the object, car stopped), WATCH_AHEAD / WATCH_LEFT / WATCH_RIGHT (point the servo there and report the distance but latch nothing — used while S3 drives round an object, when a latch would stop the manoeuvre before it began) | |
| IF-14 | S5 → S3 | published words + inline test (owned by S5) | `barcode_edge_us`, `barcode_edges`, `barcode_crossing()` | "a bar is going past the barcode sensor right now" (an edge less than `S5_CROSSING_US` ago). S3 softens its steering and holds off a spin while it is true, so the pivot does not whip the sensor across a code | |
| IF-13 | S6 → S1, S3 | shared data + mutex (owned by S6) | `mtx_obs`, `obstacle_get_status()` | `OBS_STATUS`: filtered distance, state, the latch, and the scan: the fan, the object's measured length and edge angles, the clearance either side | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | S6 |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | 1 | 1 |
| Semaphore | 16 | | | | | | |
| Event flag | 16 | | | | 1 | | 2 (`flg_obs`, plus `flg_echo` inside `drv_ultrasonic.c`) |
| Mailbox | 12 | | | | | | |
| Mutex | 12 | | 1 | 1 | 1 | | 1 |
| Message buffer | 12 | 1 | 1 | 1 | 1 | | 1 |
| Fixed memory pool | 12 | | | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | | | | | | |
| Alarm handler | 12 | | | | | | |
| Interrupt handler | — | | | | | 1 (TIMER_IRQ_3) | 1 (IO_IRQ_BANK0) |

(The kernel's USB and WiFi service tasks also use a few of these.)

## 7. Changes to shared (template) files

Agreed for the car. Each one is marked "Car" or "TEAM.md section 7" in the file.

| # | File | Change | Why |
|---|---|---|---|
| 1 | `config/config.h` | `CNF_TIMER_PERIOD` 10 → **1** ms | the line follower reads its sensors every 1 ms; `tk_dly_tsk(1)` must mean 1 ms |
| 2 | `build_make/pico_rp2040.mk` | defaults `NET ?= lwip`, `MQTT ?= 1` | telemetry is part of the car: a plain `make` builds it |
| 3 | `build_make/makefile` | link line ends with `-lm` | the IMU needs `atan2f` / `sqrtf` (soft float) |
| 4 | `app_program/app.h` | `<stddef.h>` + `PROHIBIT_DEF_SIZE_T` before the kernel headers | T-Kernel's own `size_t` (signed) clashes with the C library's; subsystems need `<string.h>` |
| 5 | `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | calls `comms_task()` instead of `mqtt_utk_start()` | S1's comms engine runs on the task that owns lwIP |
| 6 | `config/wifi_secrets.h` (private) | `MQTT_TOPIC` added | only so the unused template demo `mqtt_utk.c` still compiles |
| 7 | `kernel/sysdepend/pico_rp2040/cpu_clock.c` | sets the watchdog tick to clk_ref / 12 (1 µs) | **template bug**: the RP2040 TIMER (`time_us_64()`) ran ~8× slow, so every timing based on it (steering, speed, lift detection, barcode sampling, WiFi driver delays) was 8× slow. Tell the instructor: it affects every team using this template. |

**Reset rule (found on the car):** unlike a Pico SDK program, the kernel
takes only some hardware blocks out of reset at boot
(`kernel/sysdepend/pico_rp2040/hw_setting.c`: IO, pads, UART0, PIO, DMA,
ADC, I2C0). A block in reset ignores every write. Any other block a driver
uses must be released by that driver: `drv_motor.c` does it for **PWM**,
`drv_i2c.c` for **I2C1**. Symptom if forgotten: the code "runs" (states
change on the status line) but the hardware does nothing.

**Header rule (found while porting):** a file that includes a Pico SDK
hardware header (`hardware/...`) must **not** include `app.h` or any `tk/`
header: the kernel's `sysdef.h` defines register names (`PWM_BASE`,
`TIMER_BASE`, ...) that clash with the SDK's. That is why every driver
(`drv_*.c`) is plain C with a plain-C header, and pins live in `sN_pins.h`.
