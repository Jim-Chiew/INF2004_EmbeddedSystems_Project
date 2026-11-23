# Subsystem 1: WiFi / MQTT communication and telemetry (Buddy 1)

What it does: connects the Pico W to WiFi and an MQTT broker, sends the car's
state, speed, distance, object length, direction, hump, tilt and roll as one 10-byte binary frame 5 times a second, keeps up to
100 messages when the link is down, and receives START / STOP / RESET commands.

**micro T-Kernel build (this project).** The work is split over two tasks:

| Task | Runs | Calls |
|---|---|---|
| kernel WiFi task `cyw43_utk_task` (core 0, priority 4) | every 2 ms | `comms_task()`: connect, publish, receive (the only task that may touch lwIP/MQTT, TEAM.md section 3) |
| `telemetry_task` (this subsystem, core 0, priority 18) | every 10 ms | `comms_led_task()`, `comms_get_command()` → navigation (IF-07), `comms_send_barcode()` for each pass from S5 (IF-09), `comms_send()` every 200 ms, status line every 250 ms |

`comms_init()` is called from `subsystem1_init()`. The engine and the plain-C
parts are unchanged from the Pico SDK version; only the port file is new
(`comms_port_utk.c`), and `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`
calls `comms_task()` (TEAM.md section 7). The status LED is an external LED on
GP19: the Pico W's own LED is on the radio, which no application task may touch.

Message layout (bit positions, units, ranges, invalid codes) is in
[data_layout.h](data_layout.h). Read that first. Topics: `robocar/car1/telemetry`
(car to broker, QoS 1, 10-byte binary frame), `robocar/car1/obstacle` (car to
broker, one line per servo scan), `robocar/car1/barcode` (car to broker,
QoS 1, the decoded barcode text as plain ASCII without the `*`s, e.g. `A`, sent once
per barcode read; the frame only has room for LEFT/RIGHT) and `robocar/car1/cmd`
(broker to car, 1 byte: 1 START, 2 STOP, 3 RESET).

## Files

| File                                          | What it is                                                                                        | Needs the Pico? |
|---|---|---|
| `comms.h`, `comms.c`                          | The interface for the team: `comms_init()`, `comms_send()`, `comms_send_barcode()`, `comms_task()`, `comms_get_command()` | Needs the port file above and lwIP MQTT |
| `comms_port.h`                                | The list of platform functions that `comms.c` calls (WiFi, clock, LED, lock)                      | No |
| `comms_port_utk.c`                            | The micro T-Kernel version of those functions (link state from `cyw43_utk_get_status()`, LED on GP19) | Yes (kernel WiFi port) |
| `subsystem1.c/.h`, `task_telemetry.c`         | This subsystem's kernel objects, `telemetry_task`, the frame builder and the status line          | Yes (micro T-Kernel) |
| `subsystem1_test.c`                           | On-target tests S1-U62..S1-U64 (`make TEST=1`)                                                    | Yes |
| `drv_ws2812.c/.h`                             | The Robo Pico's two RGB LEDs on GP18 (PIO), used as the error indicator (colours in `task_telemetry.c`) | Yes (Pico SDK PIO) |
| `trans_queue.c/.h`                            | 100-message send queue, drops the oldest when full, counts drops (FR-COM-09 to 12)                | No, plain C |
| `com_state.c/.h`                              | COM_DISCONNECTED / COM_SYNC / COM_CONNECTED (FR-COM-01, 03, 04)                                   | No, plain C |
| `status_led.c/.h`                             | LED on or off for a state and time (FR-COM-05, 06)                                                | No, plain C |
| `data_layout.h`                               | The one shared table: field positions, widths, units, invalid codes, command codes                | No, constants only |
| `telemetry.c/.h`                              | Packs / unpacks the 10-byte frame and decodes the 1-byte command (FR-COM-07, NFR-RES-03)          | No, plain C |
(`comms_link.c` and `comms_port_sdk.c`, the plain Pico SDK versions, stay in the old LineFollow project.)

## How the other subsystems use it

They do not call comms functions. Navigation gets START / STOP / RESET from
`mbf_nav_cmd` (IF-07); subsystem 5 sends each barcode pass to `mbf_barcode`
(IF-09). `telemetry_task` reads everyone's published status (IF-03, IF-06,
IF-08) to build the frame. This code never touches the motors.

## How to test it

1. **On a PC (no Pico needed):** `sh testing/subsystem1/run_pc_tests.sh` from the project root (see `testing/subsystem1/README.md`).
2. **On the car:** `make TEST=1 -j8` in `build_make` runs S1-U62..S1-U64; the bench tests are in `testing/subsystem1/TEST_LOG.md`. WiFi settings go in `config/wifi_secrets.h` (copy `config/wifi_secrets.example.h`).

Results are in `testing/subsystem1/TEST_LOG.md`.

## Known open items

- No password or TLS yet (NFR-SEC-01/02 not met).
- The frame carries no battery voltage, distance or timestamp (Minimum-Bit change). A 4-bit sequence number and a drop flag replace the timestamp.
- Link-loss time (about 4 s worst case, shown on the PC clock) is not measured
  on the car with a real hotspot-off test yet. The `comms_task()` running time
  was measured on the bench: about 1.1 ms normally, 15.2 ms once during a
  connect attempt with the broker off (see `testing/subsystem1/TEST_LOG.md`).
- A command sent in the first 300 ms after the car subscribes is ignored on purpose
  (stale retained START). Resend it.
- `hump_h` carries the hump jolt (peak vertical acceleration, 0.02 g steps, 0-2.56 g), not a height in mm.