# Subsystem 1: WiFi / MQTT communication and telemetry (Buddy 1)

What it does: connects the Pico W to WiFi and an MQTT broker, sends the car's
state, speed, distance, heading and battery voltage once a second, keeps up to
100 messages when the link is down, and receives START / STOP / RESET commands.

Message format, topics and how the other tasks use it: see
[COMMS_FORMAT.md](COMMS_FORMAT.md). Read that first.

## Files

| File | What it is | Needs the Pico? |
|---|---|---|
| `comms.h`, `comms.c` | The interface for the team: `comms_init()`, `comms_send()`, `comms_task()`, `comms_get_command()` | Needs the port file above and lwIP MQTT |
| `comms_port.h` | The list of platform functions that `comms.c` calls (WiFi, clock, LED, lock) | No |
| `testing/subsystem1/bench/comms_port_sdk.c` | The Pico SDK version of those functions. CMake must define `COMMS_PORT_SDK` | Yes (Pico SDK, cyw43) |
| `trans_queue.c/.h` | 100-message send queue, drops the oldest when full, counts drops (FR-COM-09 to 12) | No, plain C |
| `com_state.c/.h` | COM_DISCONNECTED / COM_SYNC / COM_CONNECTED (FR-COM-01, 03, 04) | No, plain C |
| `status_led.c/.h` | LED on or off for a state and time (FR-COM-05, 06) | No, plain C |
| `telemetry.c/.h` | Builds the JSON message, max 256 bytes (FR-COM-07, NFR-RES-03) | No, plain C |

## How to use it from another task

```c
comms_init(&cfg);                    // once, at start-up (does not block)
...
telemetry_sample_t s = { ... };      // fill from Odometry, IMU, Nav, Safety
comms_send(&s);                      // once a second
comms_task();                        // every ~10 ms
comms_led_task();                    // every ~10 ms
comms_cmd_t c = comms_get_command(); // START / STOP / RESET or COMMS_CMD_NONE
```

Call all of these from one task. `comms_task()` never blocks. This code never
touches the motors: Nav reads the command and decides what to do.

## How to test it

1. **On a PC (no Pico needed):** see `testing/subsystem1/README.md`.
2. **On the Pico W:** see `testing/subsystem1/bench/README.md`.

Results are in `testing/subsystem1/TEST_LOG.md`.

## Known open items

- No password or TLS yet (NFR-SEC-01/02 not met).
- Battery voltage is not available yet (`bat` = 0).
- START / STOP need an IDLE state in Nav (agreed in chat, not in the chart yet).
- Link-loss time (about 4 s worst case, shown on the PC clock) is not measured
  on the car with a real hotspot-off test yet. The `comms_task()` running time
  was measured on the bench: about 1.1 ms normally, 15.2 ms once during a
  connect attempt with the broker off (see COMMS_FORMAT.md section 9).