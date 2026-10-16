1. How the parts talk to each other
 Odometry task --(copy under mutex)--> speed, distance, heading --+
 IMU task      --(copy under mutex)--> peak Y-accel, hump, shock -+--> Telemetry_Comms_Task
 Nav task      --(PROPOSED: copy)----> current state ------------+     (comms.c)
 Safety task   --(PROPOSED: copy)----> battery voltage ----------+        |
                                                                          | WiFi / MQTT
 Nav task  <--(comms_get_command)-- START / STOP / RESET <----------------+--- broker
 UI_LED task / LED  <-- COM state (CONNECTED / SYNC / DISCONNECTED)
Telemetry_Comms_Task reads the other tasks' data. It never writes to them.
Telemetry_Comms_Task never touches the motors. A command is only reported through comms_get_command(). Nav decides what to do.
Comms problems (no WiFi, no broker) never stop the car. Messages wait in a 100-message queue (oldest dropped when full, and counted).
2. The four calls (comms.h)
Call	What it does
comms_init(&cfg)	Starts WiFi + MQTT. Returns at once; the connection is made by comms_task().
comms_send(&sample)	Formats one sample as JSON and puts it in the send queue.
comms_task()	Call every ~10 ms. Reconnects (every 5 s), sends queued messages, receives commands. Never blocks.
comms_get_command()	Returns START / STOP / RESET once, or COMMS_CMD_NONE.

Also: comms_led_task() (onboard LED), comms_get_state(), comms_get_stats(), comms_now_ms().

Call all of them from one task. If a second task must call comms_send(), put a mutex around comms_send() and comms_task().

A working example is testing/subsystem1/bench/buddy1_mqtt.c.

3. MQTT topics
Topic	Direction	QoS	Content
robocar/car1/telemetry	car -> broker	1	one JSON telemetry message per second
robocar/car1/cmd	broker -> car	1	plain text command

Topic names are #defines at the top of comms.h. Change them there only.

4. Telemetry message (car -> broker)

One JSON object per message, whole numbers only, at most 256 bytes (NFR-RES-03):

{"ts":123456,"state":"LINE_FOLLOW","spd":250,"dist":1830,"hdg":905,"bat":7420}
Field	Meaning	Unit	Type	Comes from
ts	time since power-up	ms	uint32	comms (comms_now_ms())
state	Nav state name	text, letters/digits/_ only, max 23 chars	string	Nav (PROPOSED)
spd	speed	mm/s	int32	Odometry
dist	distance travelled	mm	int32	Odometry
hdg	heading	degrees x 10 (905 = 90.5 deg)	int16	Odometry
bat	battery voltage; 0 = not available	mV (7420 = 7.42 V)	uint16	Safety (OPEN: nobody reads the battery yet, see section 9)

Not in the message yet (PROPOSED, needs agreement with Odometry/IMU): validity flag for each of speed / distance / heading, peak Y-accel, hump count, shock count (FR-PER-15).

Delivery: QoS 1 means at least once. A message can arrive twice after a reconnect. Use ts to spot duplicates. Messages are sent oldest first.

5. Commands (broker -> car)

Publish the plain text below to robocar/car1/cmd. The text must match exactly: upper case, no spaces, nothing extra.

Text	Meaning (agreed in the group chat on 16 Oct; the Nav buddy adds it to the Nav statechart)
START	Start the race (leave IDLE), or move again after a STOP
STOP	Stop the car and wait in IDLE (no fault)
RESET	Leave FAULT_STOP and go back to CALIBRATE (FR-SAF-03)

Nav needs a new IDLE state: CALIBRATE -> IDLE, START -> LINE_SEARCH, any running state -> STOP -> IDLE. FR-INI-02 changes with it.

Anything else (wrong case, extra characters, longer than 8 bytes, empty) is rejected: no command is created and cmds_rejected goes up by one. Messages on other topics are ignored.

Up to 3 unread commands are kept. A fourth is dropped and counted as rejected.

Commands are acted on by the receiver of comms_get_command() (Nav). FR-COM-08 asks for action within 200 ms of receipt: comms reports the command as soon as it arrives, so Nav must read it at least every 20 ms.

6. Connection behaviour
WiFi or broker lost: the state becomes COM_DISCONNECTED, the LED blinks at 2 Hz, samples keep going into the queue (max 100).
A broker that stops answering is noticed like this: a QoS 1 message gets no ack within 3000 ms (COMMS_ACK_TIMEOUT_MS), so the link is treated as lost and a reconnect starts. A sample is published at least once a second, so the loss is seen within about 1 s + 3 s (FR-COM-01, about 4 s). On the PC test it was 4.01 s in the worst case. Not yet measured on the car with a real unclean loss.

Why 3 s and not 0.9 s: on the bench WiFi, 0.9 s gave a false "link lost" about once per 40 s (9 events in about 360 s). 3 s gave 1 event in about 10.7 minutes (one run, dropped=0). The cost is slower detection of a real loss.
7. Settings the team must set

In testing/subsystem1/bench/wifi_secrets.h (private, not committed; copy wifi_secrets.example.h): WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID.

The broker IP is the IP of the computer running Mosquitto on the same WiFi network as the Pico W.

8. How to test it (needs a Pico W, WiFi and Mosquitto)
Start Mosquitto on the laptop (config: listener 1883 and allow_anonymous true, bench only).
Flash buddy1_mqtt. Watch the serial monitor: state=CONNECTED.
See the messages arrive. In a PowerShell window: & "C:\Program Files\mosquitto\mosquitto_sub.exe" -h localhost -t "robocar/#" -v One line per second should appear.
Send a command. In another window: & "C:\Program Files\mosquitto\mosquitto_pub.exe" -h localhost -t robocar/car1/cmd -m START The serial monitor should print COMMAND RECEIVED: START.
Send a bad one (-m start, -m GO). Serial: cmds_bad goes up, no command.
9. Known limits
No password and no TLS yet (NFR-SEC-01 and NFR-SEC-02 are not met). The config has username/password fields for later; this is not tested.
MQTT keep-alive is 10 s, not the 1 s in NFR-STD-02. Link loss is detected by the ack timeout above instead (3 s). The requirement text should change to match. A WiFi stall longer than 3 s causes a reconnect, and the queued message is sent again, so a message can arrive twice (ignore a repeated ts).

The 4 s link-loss time is shown on the PC (fake clock) but not yet on the car with a real hotspot switched off.
Running time of comms_task() on the board (bench, plain SDK, 10 minute run): the longest call was about 1.1 ms in normal running, and 15.2 ms once, during a connect attempt with the broker switched off. CPU load was at most 1.0 % in any one second. One bench run, not a worst-case proof.Battery: nobody reads it yet, so bat is 0. The Robo Pico can sense the battery on GP28 only if a jumper on the back is soldered, and the reading is half the battery voltage (Safety must multiply by 2). GP28 is also used by the barcode sensor.