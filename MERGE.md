# What this build is

**V5, plus two navigation states, with the wheels untouched.**

Six files differ from V5, and `subsystem2/` is not among them:

| File | What changed |
|---|---|
| `subsystem5/task_barcode.c` | IF-15 counters added (`s5_reads`, `s5_fails`, `s5_cmd`). Pure addition. |
| `subsystem5/subsystem5.h` | Declares them. Pure addition. |
| `subsystem3/subsystem3.h` | `NAV_BARCODE_RESCAN`, `NAV_TURNING` appended to the enum, so existing numbers do not move. |
| `subsystem3/subsystem3.c` | Names for the two new states. |
| `subsystem1/task_telemetry.c` | Maps them to `DL_STATE_BARCODE_RESCAN` / `DL_STATE_BARCODE_EXECUTION`. |
| `subsystem3/task_nav.c` | The two states, plus **one** guard inside existing code. |

**Not touched:** `task_motion.c`, `subsystem2.h`, `avoid.h`, `avoid.c`, subsystem 6,
`testing/subsystem3/test_avoid.c`.

> **Later change to subsystem 2:** `MOTION_CMD_DRIVE` now also runs
> **backwards** (both powers 0..-100), with the same encoder sync. Nothing
> sent a negative DRIVE before (it was clamped to 0), so every forward drive
> is unchanged. The barcode rescan's reverse leg uses it: straight back with
> both wheels in step, no line-sensor steering, because steering by sensors at
> the trailing end walked the car off the line. See `bc_reverse()`.

Exactly three lines of *code* were removed from V5's `task_nav.c`: the learning
guard below, the GP20 return value below, and `nav.junctions++` moving earlier
in the same branch. Everything else removed is comment or console text.

## GP20: one button, two meanings

GP20 is now a plain toggle, and nothing else:

- **stopped** → clear every latched flag, whatever it is, and go straight into
  line following
- **running** → stop the car, from *any* state: following, avoiding, turning at
  a junction, rescanning a barcode, or part-way round an object

It never resumes, never confirms and never asks a question.

Five of the six GP20 read sites already stopped the run. The sixth was the wait
inside `NAV_AVOIDING`, where it used to clear the latch and carry on in place —
so one button meant "stop" in five states and "carry on" in that one. It now
returns `REQ_STOP` like the rest. To go again, press it a second time from the
standstill.

`START` from the broker still resumes in place. The broker has separate START
and STOP commands, so neither has to do double duty there.

The start path clears the latch and starts, with **no check on the road ahead
and no second press**. The one thing it does wait for is the clear actually
landing: `S6_FLG_OBSTACLE` lives in subsystem 6 and `OBS_REQ_CLEAR` is a
message, not a function call, so it can sit behind a servo scan for seconds.
Without `wait_obstacle_cleared()` the first pass of `follow_line()` would read
the stale latch and dive back into the manoeuvre, and the button would look
broken. It is not a check on whether the object is still there — it cannot
refuse to start, and it starts anyway if it times out. If the object really is
still in front of the car, subsystem 6 latches it again a few pings later and
the manoeuvre runs from line following, exactly as it would have anyway.

## Making the 90° turn repeatable

There is no gyroscope on this car — the LSM303D is an accelerometer and a
magnetometer — so a turn cannot be integrated as it happens. It has to be spun
open-loop by the encoders and then *checked* against the compass, which makes
the whole thing only as good as that one reading.

The turn was not biased. It was **scattered**: right on average, and any
individual turn anywhere from 75° to 100°. Three things caused that, all inside
`face()`.

**1. The reading before the spin had no settle at all.** `now = turn_heading()`
ran the instant `face()` was called, straight off the brakes. That reading is
the "before" half of the track-width learning, so a bad one walks `track_mm` —
and `track_mm` scales every spin the car makes afterwards. The learning, which
is V5's own mechanism for absorbing exactly this kind of error, was being fed
noise and chasing it.

**2. The reading after the spin got a fixed 250 ms,** and the captured run says
the field is still moving well past that: 250 ms after a drive the compass read
111, at 500 ms 115, at 750 ms 124, still climbing, against 146/146/146/147 on a
car that had stood still. The trim then aims at the wrong number.

**3. `AVOID_SPIN_EXTRA_DEG` was being applied three different ways in one
file** — added by the main spin (`deg + 40`), ignored by the trim (`deg`), and
scaled by the line search (`deg × 130/90`). It corrects a *proportional* error,
so scaling is the right form. A trim asking for a bare 13° of arc delivered
about 8, so every correction fell short by the same fraction.

### The fix

- **`heading_settled()`** replaces "wait 250 ms, take one sample". It samples
  until the last `AVOID_HEADING_SAMPLES` readings agree within
  `AVOID_HEADING_STABLE_DEG`, then returns their **median** — a median because
  one spike from a motor current moves a mean and cannot move a median. A fixed
  wait cannot be right for both a full 90° spin and a 5° trim; waiting for the
  reading to actually stop moving is right for both. `AVOID_COMPASS_SETTLE_MS`
  is kept as a floor, so nothing reads sooner than V5 did, and
  `AVOID_HEADING_MAX_MS` is the ceiling so a disturbed compass cannot stall a
  run. Used for **both** readings in `face()`.
- **`turn_arc(deg)`** is now the single rule for turning "degrees wanted" into
  "arc to ask for", used by the main spin, the trim and `ls_arc()`.

Nothing else moves. Every `face()` in the file turns ±90 — the manoeuvre's four
turns and the junction's one — and at 90 the additive and proportional forms are
the same number (`90 + 40 == 90 × 130 / 90 == 130`), so **the main spin is
unchanged to the count** and `ls_arc()` is `turn_arc()` already. Only the trim
gains anything, and `avoid.h` is not touched.

### Every heading, not just the ones being checked

Fixing the *checks* wasn't enough, because two readings the manoeuvre depends on
were still single raw samples taken after a fixed 250 ms.

**The manoeuvre's reference heading** — `h0 = turn_heading()` at the top of
`avoid_manoeuvre()` — is the worst of them. `away_h` and `in_h` are both worked
out from it **once** and then used by all four turns, so an error there isn't a
wobble on one turn: it's a constant bias on every turn of the manoeuvre, in the
same direction. And no amount of trimming can see it, because the trim is doing
exactly what it was told — aiming at a target that was wrong before the first
wheel moved. It now reads through `heading_settled()`.

**The line search's `h0` and its per-bite reads** had the same pattern and are
converted too. The per-bite read comes straight off the end of a bite, which is
exactly when a fixed quarter-second isn't enough; a bite is a small spin, so the
adaptive wait usually returns at its floor and costs little.

**A single dropped sample no longer counts as "no compass."**
`turn_heading()` returns −1 when the IMU is missing, uncalibrated, *or when its
last read simply failed* — and one failed read on an I2C bus happens. Passing
that straight out handed a momentary hiccup to `face()`, which can't tell it
from a car with no compass at all, so it skipped the entire trim and turned by
dead reckoning — on a car whose compass was fine milliseconds later. **That was
a turn silently going unverified.** `heading_settled()` now waits and asks
again, dropping its window (readings either side of a dropout aren't
consecutive), and only reports "no compass" if it can't get a clean window
inside `AVOID_HEADING_MAX_MS`.

The result is an invariant worth keeping: **the only raw `turn_heading()` left
in the file is the one inside `heading_settled()`.** Every heading the car acts
on is settled and median-filtered.

Every exit from `face()` reports what it measured, so all four manoeuvre turns
and the junction turn print their real angle:

| exit | line |
|---|---|
| landed in tolerance | `facing 88 deg, wanted 90 (off by 2) after 13 deg of trim in 1` |
| trim budget spent | `compass wants 31 deg more after 25 deg of trim -- not believing it` |
| out of attempts | `still 7 deg off after 22 deg of trim, carrying on` |
| compass lost mid-trim | `lost the compass mid-trim, carrying on` |
| no compass at all | `turned 90 deg by the encoders alone, +40 for slip` |

### What it buys

Modelled over 24 turns at the measured roll fraction, with the learning running:

| | mean | spread | `track_mm` ends at |
|---|---|---|---|
| V5 — unsettled reading, bare trim | 90.5° | **26.0°** (75.5–101.6) | 143 |
| now — settled reading, `turn_arc` | 89.2° | **3.1°** (87.0–90.1) | 152 |

That is a model, not the car, but it reproduces the reported symptom: V5's turn
is right on average and unrepeatable turn to turn. With a trustworthy reading
the learning converges on the track width that cancels the scale error instead
of wandering, and the trim closes in one correction instead of two.

**Cost:** roughly an extra second per turn — about 4–5 s across a manoeuvre.
The junction turn got *faster*: its two fixed 1500 ms waits are gone, since
`face()` now settles its own readings. `JN_COMPASS_SETTLE_MS` no longer exists.

**On the console,** every turn now always reports where it landed:
`nav:     facing 88 deg, wanted 90 (off by 2) after 13 deg of trim in 1`.
That line is the one to watch — it is how you tell whether the turns are
repeatable without a protractor.

## One reference heading, read once

The manoeuvre reads the heading the car came in on **once**, before it turns a
wheel, and everything it does afterwards is worked out from that single
reading:

```
h0 = heading_settled()            <- the only reference reading
away_h = h0 + away_deg
in_h   = h0 + in_deg
  face(away_h)  face(h0)  face(away_h)  face(in_h)  face(h0)  face(h0)
  line_search(h0)
```

That last line is the fix. `line_search()` used to read a heading of its own,
which was the car **quietly redefining straight at the very last moment**: the
reading it took was of wherever the car actually ended up, which is the
original heading *plus* whatever the final turn failed to correct. The sweep
would then centre on the error, hunt for the line around the wrong middle, and
hand back a car believing a crooked heading was straight — and the next
manoeuvre would start from those leftovers.

`line_search()` now takes the reference as a parameter:

| caller | what "straight" means |
|---|---|
| obstacle manoeuvre | `h0` — the reading taken before the first wheel moved |
| junction turn | `h0 + deg` — the heading it was just told to turn onto |
| barcode rescan | `NO_HEADING` — it never reads the compass at all, so the search takes its own |

**The readings `face()` takes after each spin are not references, they are
measurements against this one.** That distinction is what lets the car verify
every turn without drifting: the target never moves, so an error on one turn
cannot be inherited by the next.

## The rescan steers like the line follower

The rescan's corrections now use `STEER_OUTER` and `STEER_INNER` themselves —
the line follower's own constants, not copies — so the inside wheel **stops**
and the car pivots around it while the outside wheel speeds up from 10 to 35.
Tuning the follower tunes this with it.

It used to use the soft pair (21/35, both wheels driven), on the grounds that
reversing with the sensors at the trailing end is the trailer-backing problem:
the correction also swings the car further off line, so a hard pivot could
jackknife rather than settle.

### Reversing uses the same wheel as going forward

This was a real bug, and it made the reverse leg drive *away* from the line.

The rule is: **slow the wheel on the side the sensor saw the line** — right
sensor black, slow the right wheel — in **both** directions, no swap. It looks
wrong, because the nose does the opposite thing each way and the nose is what
you watch. Work it through instead, with the body turning anticlockwise
whenever the right wheel outruns the left:

| | wheels | body | nose | **car travels** |
|---|---|---|---|---|
| forward, slow right | `vl=+35 vr=0` | clockwise | swings RIGHT | **RIGHT** |
| reverse, slow right | `vl=-35 vr=0` | anticlockwise | swings LEFT | **RIGHT** |
| reverse, slow **left** ← the old code | `vl=0 vr=-35` | clockwise | swings RIGHT | **LEFT** |

Both of the first two travel right, which is where the line is. Reversing, the
car is steering its **rear** — the end that leads — so the nose swinging the
other way is the manoeuvre working, the same reason a reversing car turns its
wheel the "wrong" way into a parking space.

The old code swapped the wheels when reversing, reasoning that the yaw
direction had to be preserved. It preserved the yaw and reversed the *travel*:
a right-sensor hit slowed the left wheel, the nose obediently swung right, and
the car drove left, away from the line. Every correction made the error bigger,
so the next one was bigger again — which also accounts for a good deal of the
"jerky", since a wrong-signed feedback loop oscillates.

The fix is that the wheel choice no longer looks at `dir` at all. `dir` sets
the sign of the two powers and nothing else. Forward behaviour is unchanged.

### The ramp

**The ramp is what makes the pivot safe here.** The jackknife comes from the
suddenness, not from the inner wheel being stopped — a step change straight to
0/35 kicks the tail out faster than the sensors can report it. So the
correction is not switched on, it is wound on, now 4 % every 10 ms:

```
   ms  level     L      R      (reverse, winding on to the right)
    0      0    -10    -10     (equal -> encoder-synced)
   60     24    -16     -8
  120     48    -22     -6
  180     72    -28     -3
  250    100    -35      0     (full pivot, inner wheel stopped)
```

250 ms from straight to full pivot and the same back, in 25 steps. It was 10
steps over 120 ms, which is 2–3 % of power a step — coarse enough to feel on a
car crawling at `STRAIGHT_SPEED`. At 4 every 10 ms the outer wheel moves one
per cent at a time, 26 distinct powers on the way up. Slower also means the
correction has longer to show up in the sensors before it is fully wound on, so
the loop is less able to overshoot and start hunting.

The level is **signed**, so a correction that changes sides passes through
straight on its way instead of slamming from one pivot to the other.

And the rescan is the one place that can afford a pivot, because the path is
known straight — one code on one straight stretch of line — so every correction
is a small one against drift and the ramp has time to work.

`bc_steer()` is called every pass of the loop now (it needs the ticks) but only
talks to the motors when the powers actually change. It and `bc_steer_reset()`
are called from `bc_reverse()` and `bc_forward()` and nowhere else, so **nothing
outside the rescan is affected.** If it ever does jackknife, raise
`BC_STEER_INNER` off zero — that alone turns the pivot back into a curve with
the ramp still in place.

## Coming back to the line: a deliberate overshoot

> **Replaced.** The overshoot below left the axle 5 cm past the line, and a
> spin on the spot from there can only reach the line by pointing the car
> about 25° *across* it. The search stopped with the line under its trailing
> sensor, follow_line() drove a few mm to confirm, lost it, and drove off
> beside the line. The return leg now watches the line sensors and stops with
> the **axle over the line** (`JN_ADVANCE_MM` past the first sighting, the
> junction turn's figure), so the turn pivots on the line. The constant is
> now `AVOID_RETURN_SEEK_TRUE_MM` (150): how far past the distance stepped out
> to keep looking before falling back to the old turn-and-sweep. See
> `avoid_return_to_line()` in `task_nav.c`.
>
> **And after that:** the creep is now `S3_SENSOR_TO_AXLE_TRUE_MM`, a ruler
> measurement (default 117, the old 65 % of 180), instead of a percentage of a
> guessed car length. If it is too big, the creep puts the axle out of the
> sweep's reach and the search finds nothing either side;
> `avoid_rejoin_from_far_side()` then drives at the line again from the other
> side and measures the creep the car really needs, `(p - t) / 2`, printing it.
> The search now stops when the line is **between** the sensors (the leading
> one has had it and let go), not when it reaches the trailing one. And
> `S3_AVOID_USE_COMPASS 0` keeps the compass out of the obstacle manoeuvre,
> because positioning was better without a GP21 calibration than with one.
>
> The original note follows.

The way back drives exactly what was stepped out, so on paper the car arrives
back on the line it left. On the floor it doesn't quite — four turns, a long
drive and a dragged tyre each lose a little, and all of it lands on that last
leg. Falling **short** is the expensive way to be wrong: the car stops with the
line still ahead of it, the sweep has to hunt for it, and a sweep that finds
nothing ends the run.

So the return leg now aims **5 cm past** the line:

```c
-	r = avoid_straight(stepped, TRUE);
+	r = avoid_straight(stepped + AVOID_RETURN_EXTRA_MM, TRUE);
```

Crossing the line costs nothing. The search that follows sweeps the nose either
way and finds a line just behind the sensors as readily as one just in front,
and the front guard is on for this leg, so anything appearing ahead still stops
the car.

**It goes through `S3_TRUE_MM`,** because 5 cm is a ruler measurement and not
one of `avoid.h`'s tuned figures. Handing the car a bare 50 would move it about
30 mm. Measured:

| | |
|---|---|
| `AVOID_RETURN_EXTRA_TRUE_MM` | 50 (the ruler) |
| asked for | 84 |
| real floor moved | 50 mm |

The return leg therefore grows from 170–470 to 254–554 in asked-millimetres.
Its stall timeout is 56 s against the ~3.3 s the longest version needs, so there
is no risk of the longer leg tripping it. To change the overshoot, change the
TRUE figure; the constant lives in `task_nav.c`, so `avoid.h` is still untouched.

The console reports both scales:
`nav: back at the line, 170 mm out and 254 mm back (170 stepped + 84 of deliberate overshoot, 5 cm on the ruler)`

## The learning guard inside V5's own code

```c
-	if (deg >= AVOID_TURN_TRUST_MIN_DEG) {
+	if (deg >= AVOID_TURN_TRUST_MIN_DEG && face_learn) {
```

`face_learn` is a file-scope `BOOL` that is `TRUE`. `junction_turn()` clears it
immediately before its `face()` call and restores it immediately after, before
the result is even looked at, so no path out can leave it off.

This is what fixes the 90-degree regression. `track_mm` is the learned wheelbase that
`avoid_turn_arc_track()` computes every 90° obstacle spin from, and `face()` is
the only thing that writes it. A junction turn is 90°, so without this guard it
votes on that wheelbase like any other turn — and it is a far worse sample,
judged on two magnetometer readings taken mid-course. `avoid_track_update()`
moves `track_mm` a quarter of the way toward whatever they imply, so a few
junctions into a run the manoeuvre is spinning on a wheelbase the barcodes
taught it. That is why the 90° turns stopped coming out at 90.

V5 still learns from its own turns exactly as it always did. A junction simply
does not get a vote.

## The line search never starts a rescan

A sweep pivots the car on the spot, which drags the barcode sensor sideways
across the very black line it is hunting for. Subsystem 5 can't tell that from
a car driving over a printed code — it sees bars and gaps, starts a pass, and
when the pass won't decode (it won't, a line isn't a code) it steps `s5_fails`.
`follow_line()` reads that as "a code failed" and reverses back up the line
looking for a barcode that was never there. And because a rescan *ends* in
another line search, that shakes out another failure, which starts another
rescan.

**Syncing the counters when the search returns is not enough**, and that's the
trap. A pass doesn't finish when the bars stop going past — it finishes
`BARCODE_END_GAP_MS` (a whole second) after its **last edge**. So the sweep
ends, the counters are taken as read, the car goes back to following, and a
second later the pass the sweep started lands in `s5_fails` and trips the
rescan anyway. Only a window that outlasts the pass closes that door.

So `line_search()` mutes the rescan for `BARCODE_END_GAP_MS + 500` ms, set at
the one point after the last bite and before all four of its returns. A failure
arriving inside that window is logged and dropped.

**Reads are not muted.** A sweep cannot fake a Code 39 decode — that needs the
start and stop characters and credible bar ratios — so a read arriving here is
a real code and still sets the command for the next junction.

One mute inside `line_search()` covers every caller, because all three routes
back to the line go through it: the junction turn, the rescan's own tail, and
the end of the obstacle manoeuvre. That last one had no counter sync at all
before, so it was the widest way in.

## Interference audit

`task_nav.c` has three pieces of file-scope state the manoeuvre depends on:

- **`track_mm`** — written only in `face()`, now guarded. The junction *reads*
  it (through `face()` and `spin_arc()`), which is harmless. `line_search()`
  reads it too, and never wrote it even in V5.
- **`step`** — written only by `set_step()`, which only the manoeuvre calls.
  Read only for a console message behind the front guard, which the new states
  do not run.
- **`plan`** — written only inside the manoeuvre.

Everything else the junction shares with the manoeuvre (`face()`, `spin_arc()`,
`avoid_move()`, `avoid_drive()`) is stateless code. **So both states are enabled
in this build** (`S3_JUNCTION_TURN 1`).

## Obstacles are line-following's business only

`avoid_obstacle()` is entered from exactly one place: the latch check at the
top of `follow_line()`. Neither new state runs the front guard — all five of
their check calls pass `FALSE`, and the only `avoid_checks(TRUE)` calls left in
the file are V5's own, inside `avoid_object()`.

Nothing is lost by that. Subsystem 6 **latches** what it sees, so an object
spotted during a rescan or a turn is still sitting in the flag when the state
hands back to `follow_line()` a fraction of a second later — where the
manoeuvre picks it up exactly as if the car had never left the line.

Safety is checked throughout both states: lifted, overturned, STOP and RESET
all end them immediately.

## How the states work on V5's wheels

**Driving.** `avoid_drive()` is byte-for-byte V5's. Its rule is that equal
powers *above zero* go to the encoder-synced `MOTION_CMD_DRIVE` and everything
else goes straight to `MOTION_CMD_WHEELS`, which V5's `drive_wheels()` already
accepts at −100..100. So:

- forward and straight → `DRIVE`, synced, as line following always was
- backward and straight → `WHEELS`, **not** synced
- any steering correction → `WHEELS`, not synced, which is correct — a curve is
  meant to happen

The middle one is the only compromise in this build. Reversing unsynced lets
the stronger motor walk the car slightly off line, and the line sensors correct
that continuously through the whole leg, which is how it is steered anyway. The
alternative was teaching `MOTION_CMD_DRIVE` to sync backwards — a change to
subsystem 2 that every other user of `DRIVE` inherits, the manoeuvre included.
Not worth it for one leg that has a sensor watching it.

**Distances.** The ported code's lengths came off a ruler — A4 is 297 mm, the
car is about 180 mm — while V5's odometry under-reads by about 1.69×. Handed
straight over, a 297 mm reverse leg over a 297 mm code stops the sensor in the
middle of the barcode. `S3_TRUE_MM()` converts them, and is used by these two
states and nothing else:

| | ruler | asked for |
|---|---|---|
| barcode code | 297 | 502 |
| clear-floor backstop | 40 | 67 |
| reverse cap | 1000 | 1692 |
| car length | 180 | 304 |
| junction creep | 117 | 198 |

A reverse leg that starts 100 past the code asks for 689 and moves 407 mm of
real floor, where 371 is needed — a 10 % margin, with the `BC_CLEAR_MM` sensor
backstop as the real guarantee.

`S3_TRUE_MM` is a ratio against `ENC_COUNTS_PER_REV`, so if the odometry is ever
calibrated properly it collapses to a no-op on its own. That also replaced a
`BC_BACK_SCALE_PCT` of 145 %, which had been floor-tuned as a partial
correction for this same scale error; it is back to 110 %, a real margin.

**The compass.** `AVOID_COMPASS_SETTLE_MS` stays at V5's 250 ms. The junction
has its own `JN_COMPASS_SETTLE_MS` of 1500 ms, used twice — before reading the
heading it came in on, and before `face()` takes its own reading. The
magnetometer is unusable for about a second after the motors stop; a junction
turn comes straight off the end of line following, so it needs the longer wait,
and giving new code one costs the manoeuvre nothing.

## Switches

| | | |
|---|---|---|
| `S3_JUNCTION_TURN` | `task_nav.c` | `0` restores V5 exactly: drive straight across a junction, sensors ignored. |
| `BC_RESCAN_TRIES` | `task_nav.c` | Back-and-forward attempts at an unreadable code. |
| `S3_ODOM_TRUE_CPR` | `task_nav.c` | Measured counts per wheel turn. Re-measure the car and change this, not the individual lengths. |
| `JN_COMPASS_SETTLE_MS` | `task_nav.c` | Settle before the junction's compass readings. |

## Built and tested

`make` builds clean with no warnings
(`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`). `test_avoid` (39),
`test_lift_gate` (15) and `test_barcode_decode` (15) all pass — `test_avoid`
against V5's unmodified `avoid.h`.

None of that replaces driving the car. Worth watching on the floor: whether the
obstacle manoeuvre's 90° turns stay square across a run with junctions in it
(they should — nothing can move `track_mm` but the manoeuvre itself), and
whether the junction creep and the rescan's margins suit the course.
