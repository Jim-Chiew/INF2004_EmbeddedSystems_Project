# Subsystem 1 - test log

These are unit tests that run on a PC (no Pico needed). They check the plain-C logic against the requirements in the design review document. Tests that need the real Pico, the WiFi network or the kernel are not in this file yet.

## Test cases

Test IDs never change and are never reused.

| ID | What is tested | Requirement | Method | Where |
|---|---|---|---|---|
| S1-U01 | queue starts empty, discard counter 0 | FR-COM-09, FR-COM-11 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U02 | peek and ack on an empty queue are refused | robustness | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U03 | 100 messages stored, none dropped (FR-COM-09) | FR-COM-09 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U04 | 101st message drops the oldest and counts it (FR-COM-10, FR-COM-11) | FR-COM-10, FR-COM-11 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U05 | oldest message is now number 2 | FR-COM-10 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U06 | discard counter keeps counting | FR-COM-11 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U07 | NULL pointer and zero length are rejected | robustness | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U08 | 257-byte message rejected (NFR-RES-03) | NFR-RES-03 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U09 | exactly 256 bytes accepted (NFR-RES-03) | NFR-RES-03 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U10 | ack of a message that was already dropped is refused | FR-COM-12 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U11 | a refused ack does not reset the discard counter | FR-COM-12 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U12 | 100 messages sent oldest first, in order | FR-COM-03, FR-COM-04 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U13 | discard counter kept until the last message is acked | FR-COM-12 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U14 | counter reset only after the queue is fully sent (FR-COM-12) | FR-COM-12 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U15 | order is kept when the queue wraps around | FR-COM-09, FR-COM-10 | auto | `testing/subsystem1/test_trans_queue.c` |
| S1-U20 | starts in COM_DISCONNECTED at power-up | Section 7 assumption (a) | auto | `testing/subsystem1/test_com_state.c` |
| S1-U21 | WiFi up but MQTT down: stays COM_DISCONNECTED | FR-COM-03 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U22 | MQTT up but WiFi down: stays COM_DISCONNECTED | FR-COM-03 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U23 | both up, queue not empty: COM_SYNC (FR-COM-03) | FR-COM-03 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U24 | stays in COM_SYNC while queue has messages | FR-COM-03, FR-COM-04 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U25 | queue becomes empty: COM_CONNECTED (FR-COM-04) | FR-COM-04 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U26 | stays in COM_CONNECTED | FR-COM-04 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U27 | WiFi lost in COM_CONNECTED: COM_DISCONNECTED (FR-COM-01) | FR-COM-01 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U28 | both up, queue empty from DISCONNECTED: COM_CONNECTED (FR-COM-03) | FR-COM-03 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U29 | MQTT lost in COM_CONNECTED: COM_DISCONNECTED (FR-COM-01) | FR-COM-01 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U30 | WiFi lost in COM_SYNC: COM_DISCONNECTED (FR-COM-01) | FR-COM-01 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U31 | MQTT lost in COM_SYNC: COM_DISCONNECTED (FR-COM-01) | FR-COM-01 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U32 | reconnects with empty queue: COM_CONNECTED | FR-COM-03 | auto | `testing/subsystem1/test_com_state.c` |
| S1-U33 | NULL inputs change nothing | robustness | auto | `testing/subsystem1/test_com_state.c` |
| S1-U34 | com_get_state() matches the last update | robustness | auto | `testing/subsystem1/test_com_state.c` |
| S1-U40 | solid ON in COM_CONNECTED and COM_SYNC for 10 s (FR-COM-05) | FR-COM-05 | auto | `testing/subsystem1/test_status_led.c` |
| S1-U41 | COM_DISCONNECTED: ON for the first 250 ms | FR-COM-06 | auto | `testing/subsystem1/test_status_led.c` |
| S1-U42 | COM_DISCONNECTED: OFF for the next 250 ms | FR-COM-06 | auto | `testing/subsystem1/test_status_led.c` |
| S1-U43 | pattern repeats every 500 ms | FR-COM-06 | auto | `testing/subsystem1/test_status_led.c` |
| S1-U44 | 20 blinks (40 changes) in 10 s = 2 Hz (FR-COM-06) | FR-COM-06 | auto | `testing/subsystem1/test_status_led.c` |
| S1-U45 | 50% duty cycle over 10 s | FR-COM-06 | auto | `testing/subsystem1/test_status_led.c` |
| S1-U46 | unknown state: LED off | robustness | auto | `testing/subsystem1/test_status_led.c` |
| S1-U50 | normal sample gives the expected message (FR-COM-07/09/13) | FR-COM-07, FR-COM-09, FR-COM-13 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U51 | message is at most 256 bytes (NFR-RES-03) | NFR-RES-03 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U52 | worst-case values still fit in 256 bytes | NFR-RES-03 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U53 | negative values are written correctly | FR-COM-07 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U54 | all-zero sample works | FR-COM-07 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U55 | buffer with room for the end marker works | NFR-RES-03 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U56 | buffer one byte too small is refused | NFR-RES-03 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U57 | small buffer: nothing written past its end | NFR-RES-03 | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U58 | NULL pointers and size 0 are rejected | robustness | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U59 | state name with a quote is rejected | robustness | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U60 | empty state name is rejected | robustness | auto | `testing/subsystem1/test_telemetry.c` |
| S1-U61 | state name without an end marker is rejected | robustness | auto | `testing/subsystem1/test_telemetry.c` |
| S1-S01 | power-up with no link: COM_DISCONNECTED | FR-COM-03 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S02 | COM_DISCONNECTED: LED is blinking | FR-COM-06 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S03 | 3 samples while offline: stored, none sent | FR-COM-09 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S04 | link back with backlog: COM_SYNC | FR-COM-03 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S05 | COM_SYNC: LED stays on | FR-COM-05 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S06 | backlog drained: COM_CONNECTED, queue empty | FR-COM-04 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S07 | broker receives samples 1, 2, 3 in order | FR-COM-03, FR-COM-04 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S08 | link lost: back to COM_DISCONNECTED | FR-COM-01 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S09 | 105 offline samples: 100 kept, 5 discarded | FR-COM-09, FR-COM-10, FR-COM-11 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S10 | reconnect after overflow: COM_SYNC | FR-COM-03 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S11 | overflow backlog drained: COM_CONNECTED | FR-COM-04 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S12 | broker gets the newest 100, oldest 5 dropped | FR-COM-10 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S13 | broker order is increasing | FR-COM-04 | auto | `testing/subsystem1/test_scenario.c` |
| S1-S14 | discard counter reset after full transmission | FR-COM-12 | auto | `testing/subsystem1/test_scenario.c` |
| S1-H01 | Pico W joins WiFi and gets an IP | FR-COM-03 | manual (Pico W) | `testing/subsystem1/bench/buddy1_wifi.c` |
| S1-H02 | MQTT connects and QoS1 publish is acked by the broker | FR-COM-03, FR-COM-04 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
| S1-H03 | broker down: queue grows, state COM_DISCONNECTED | FR-COM-01, FR-COM-09 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
| S1-H04 | broker back: queue drained in order of ack count, COM_CONNECTED, no drops | FR-COM-04, FR-COM-12 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
| S1-H05 | queue overflow drops oldest and counts them | FR-COM-10, FR-COM-11 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
| S1-H06 | MQTT retry every 5 s (measured 5002 ms) | FR-COM-02 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
| S1-H07 | LED solid ON when connected, blinking when broker stopped (by eye) | FR-COM-05, FR-COM-06 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
| S1-H08 | LED blink rate 2 Hz within 10% (59 blinks in 30 s = 1.97 Hz) | FR-COM-06 | manual (Pico W) | `testing/subsystem1/bench/buddy1_mqtt.c` |
*Method:* **auto** = a check in the test program, result printed on the console. *robustness* means the test protects against bad input and does not map to one requirement.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test IDs | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-12 | S1-U01 to S1-U15 | `933b1c7` | Sarfaraz | PASS (15/15) | `logs/2026-10-12_unit.txt` | PC build, gcc, strict warnings |
| 2026-10-12 | S1-U20 to S1-U34 | `ab73ddc` | Sarfaraz | PASS (15/15) | `logs/2026-10-12_com_state_unit.txt` | PC build, gcc, strict warnings |
| 2026-10-12 | S1-U40 to S1-U46 | `c781c91` | Sarfaraz | PASS (7/7) | `logs/2026-10-12_status_led_unit.txt` | PC build, gcc, strict warnings |
| 2026-10-12 | S1-U50 to S1-U61 | `eda6118` | Sarfaraz | PASS (12/12) | `logs/2026-10-12_telemetry_unit.txt` | PC build, gcc, strict warnings |
| 2026-10-12 | S1-S01 to S1-S14 | `e6b75eb` | Sarfaraz | PASS (14/14) | `logs/2026-10-12_scenario.txt` | PC build, gcc, strict warnings, ASan/UBSan |
| 2026-10-15 | S1-H01 to S1-H06 | `TBD` | Sarfaraz | PASS (6/6) | `logs/2026-10-15_wifi_join.txt`, `logs/2026-10-15_mqtt_offline_reconnect.txt`, `logs/2026-10-15_mqtt_retry_5s.txt` | Pico W on phone hotspot, Mosquitto on laptop, no SSID/IP in logs |

## Not yet tested
- FR-COM-08 (START/STOP/RESET within 200 ms), NFR-PRF-05, NFR-REL-03, NFR-SEC-01/02, NFR-STD-02: need the full system on the Pico.