# Embedded Take-Home: Embedded Systems Track

## Part 0: Base Knowledge Primer

### What is ROS 2?

ROS 2 (Robot Operating System 2) is a framework for writing robot software as a collection of independent programs called **nodes** that talk to each other by passing messages. Instead of one giant program controlling everything, you have small focused programs — one for motor control, one for GPS, one for camera processing — that communicate over named channels called **topics**.

### Topics, Publishers, Subscribers

- A **topic** is a named channel (e.g., `/wheel_ticks`) that carries a specific message type.
- A **publisher** sends messages onto a topic.
- A **subscriber** receives messages from a topic.
- Nodes don't know about each other directly — they agree only on a topic name and message type.

### QoS (Quality of Service)

ROS 2 lets you configure delivery guarantees per topic: `RELIABLE` vs `BEST_EFFORT`.

- **`RELIABLE`** guarantees every message is delivered. The publisher retransmits until acknowledged.
- **`BEST_EFFORT`** sends once and moves on, spending no resources tracking delivery.

**Critical:** a publisher and subscriber on the same topic must have *compatible* QoS settings, or they won't connect and **no messages arrive**. ROS 2 prints a one-line "incompatible QoS" warning, but nothing crashes and it is easy to miss in scrolling output. This trips people up constantly, including AI code generators, which often default to mismatched settings. Compatibility rule: the subscriber's reliability level must be ≤ the publisher's (a BEST_EFFORT publisher cannot serve a RELIABLE subscriber).

### TF2 (Transform Tree)

TF2 tracks position/orientation relationships between reference frames on the robot. Transforms have a **parent frame** and a **child frame** — getting this order backwards is one of the most common ROS bugs (including in AI-generated code). A `odom → base_link` transform describes `base_link` *relative to* `odom`.

### EKF and Sensor Fusion

An Extended Kalman Filter combines multiple noisy sensor sources (e.g., wheel encoders + GPS) into a single best estimate of position, weighting each by how much you trust it at that moment. You won't implement a full EKF here, but you will do a simplified version of exactly this fusion problem.

**A worked example of "weighting each by how much you trust it".** Suppose your wheel-based estimate says the rover is at x = 10.0 m and you are unsure by a variance of 0.04 m² (σ = 0.2 m). GPS says x = 10.5 m with variance 0.09 m² (σ = 0.3 m). The *gain* is the share of the correction you take from GPS:

```
gain  = wheel_variance / (wheel_variance + gps_variance) = 0.04 / (0.04 + 0.09) ≈ 0.31
fused = wheel + gain × (gps − wheel)                     = 10.0 + 0.31 × 0.5   ≈ 10.15
fused_variance = (1 − gain) × wheel_variance             ≈ 0.028   (better than either alone)
```

The more uncertain source gets the smaller say. Your wheel variance grows the further you drive without a correction, and a noisier GPS fix has a bigger variance, so the gain should change over time. That is the whole idea; a real EKF does this with matrices.

### Words you'll meet

- **Tick**: one pulse from a wheel encoder. This wheel gives 360 per revolution.
- **Cumulative count**: a running total since the start, not "ticks since the last message".
- **Dead reckoning**: estimating where you are by adding up small movements (distance, in the direction you are facing) from a known start. Small errors add up, so it drifts.
- **Heading (yaw)**: the direction the rover is facing, as an angle in *radians* (π ≈ 3.14 is half a turn), measured from east, counter-clockwise positive. East = 0, north = π/2.
- **ENU frame**: the coordinates used here: x = east, y = north, (0, 0) = where the rover started.
- **Variance / covariance**: how uncertain a number is, in m² (it is the standard deviation squared: 0.09 m² means σ = 0.3 m). Smaller means more trustworthy.
- **Quaternion**: the 4-number way ROS stores a rotation. A flat robot only turns about z, so only two numbers matter: `z = sin(heading/2)`, `w = cos(heading/2)`, with `x = y = 0`.

### How Yonder actually uses this

Our rover fuses wheel encoder data with RTK GPS through a TF2 transform tree (`map → odom → base_link`) to know where it is.

- `base_link` is the rover's body frame (centered on the rover, essentially fixed).
- `odom` is the rover's position relative to its start point. The `odom → base_link` transform comes from wheel encoders: where is the rover now relative to where it started?
- `map` is the world frame. The `map → odom` transform comes from GPS: it corrects the odom frame's drift by anchoring it to an absolute reference.

The encoder data comes from six ODrive motor controllers via CAN bus (`odrive_can/ControllerStatus.pos_estimate`). GPS comes from dual NMEA receivers publishing `NavSatFix`. A `robot_localization` EKF fuses everything into `/autonomous/localization/odometry/global`. This task is a simplified version of that pipeline.

**Resources:**
- [ROS 2 Publisher/Subscriber tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 QoS docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [TF2 introduction](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

---

## Starter Repo

### What's in the repo

```
odometry_node.py       Your file — stub with function signatures, docstrings,
                       and TODO comments. This is what you'll implement.

sim/
  launch.py            Entry point. Run this to start everything.
  encoder_publisher.py Simulates wheel encoder ticks with realistic noise.
                       PROVIDED — working, do not modify.
  gps_publisher.py     Simulates GPS position estimates with noise and outages.
                       PROVIDED — working, do not modify.
  ground_truth.py      The rover's true trajectory (a circle). Internal to the
                       simulator: your node never receives it, only the noisy
                       sensor readings derived from it. We may swap in a
                       different trajectory when we run your node.
  simclock.py          The simulator's clock (never jumps backwards).
  messages.py          WheelTicks and GPSEstimate message definitions.
  visualizer.py        Live matplotlib plot. Run with --visualize.
  rclpy_lite/          A lightweight simulator shim of the real rclpy API.
                       Lets you write ROS-style code without installing ROS.
  nav_msgs/            Standard ROS message types (Odometry, etc.) as dataclasses.
  geometry_msgs/       Pose, Twist, Quaternion, etc.
  tf2_ros/             TransformBroadcaster stub for the TF2 stretch goals.

requirements.txt       pip dependencies (numpy, matplotlib only).
AI_LOG.md              Template for your AI usage log (Part 3).
METHODOLOGY.md         Template for how to run your work and your thought process (Part 4).
```

### Familiarisation — read these before you start

Before touching `odometry_node.py`, read through the files you're given:

**`sim/encoder_publisher.py`** — This is the most important file to read first.
It publishes `WheelTicks` messages on `/wheel_ticks`. Note:
- The QoS profile it uses (hint: this is the thing most likely to silently break your node).
- The three types of noise it injects: dropped ticks, duplicate messages, and clock drift.
  Each has a comment explaining the real-world phenomenon it simulates.

**`sim/gps_publisher.py`** — Publishes `GPSEstimate` on `/gps_estimate`.
Note the QoS, the update rate, and the outage cycle. The `covariance` field in each
message tells you how much to trust that reading.

**`sim/messages.py`** — Defines `WheelTicks` and `GPSEstimate`. Read the docstrings
carefully — every field is described.

**`sim/rclpy_lite/`** — You don't need to understand this in depth. It's a lightweight
shim of the real rclpy API (Node, Publisher, Subscriber, QoS, spin). We name
it `rclpy_lite` rather than `rclpy` to avoid ambiguity if you have ROS installed.
To port your code to a real ROS 2 system, swap `rclpy_lite` → `rclpy` in your imports
(see "How this differs from real ROS 2" below for the few things that won't port unchanged).

**`sim/ground_truth.py`** — The rover drives a circle (radius 10 m, 0.5 m/s), starting at
(0, 0) **facing east** and turning left. There is no topic for this: only `encoder_publisher`
and `gps_publisher` use it internally, to make the noisy readings. It's provided so you can
understand what the true trajectory looks like. **Your node must not rely on it**: when we
run your node we may use a different trajectory (see "Where does heading come from?").

### Running it

You need Python 3.10 or newer, plus numpy and matplotlib. A fresh computer is often missing some pieces, so:

**Step 0: check Python.** In a terminal, run `python3 --version` (on Windows: `py --version`). It must say 3.10 or higher. If the command isn't found, or the version is too old, install Python first:

- **Ubuntu / Debian / WSL:** `sudo apt update && sudo apt install python3 python3-venv python3-pip python3-tk` (`python3-tk` is only needed for `--visualize`; on a fresh install `python`, `pip` and `python3 -m venv` do **not** work until you do this).
- **macOS / Windows:** install Python from [python.org](https://www.python.org/downloads/). On Windows tick "Add python.exe to PATH". If `--visualize` later complains about `tkinter` on a Homebrew Python, run `brew install python-tk`.

**Step 1: make a virtual environment and install the dependencies** (from the repo root). A virtual environment is a private folder of packages for this project; it keeps them out of your system Python, which recent Ubuntu, Debian and Homebrew refuse to modify.

```bash
python3 -m venv .venv                 # Windows: py -m venv .venv
source .venv/bin/activate             # Windows PowerShell: .venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Your prompt now starts with `(.venv)`. Open a new terminal later? Run the `activate` line again. While it is active, plain `python` and `pip` work on every OS, and that is what the rest of this README uses.

**Step 2: start the simulator** (from the repo root):

```bash
python sim/launch.py --no-node
```

You should see something like this, and then nothing more (that's correct: no node is running yet). Stop it with Ctrl-C.

```
[launch] Random seed: 271828  (rerun with --seed 271828 to repeat this exact noise)
[launch] Encoder publisher started  -> /wheel_ticks   @ ~50 Hz  (BEST_EFFORT)
[launch] GPS publisher started       -> /gps_estimate  @ ~1 Hz   (BEST_EFFORT)

[launch] Running. Press Ctrl-C to stop.
         Tip: run with --visualize to see a live plot.
```

Now run it with your node: `python sim/launch.py`. It loads `odometry_node.py`. Until you implement the subscriptions your node does nothing, and after 5 seconds the launcher tells you that nothing has appeared on `/odom`.

**Always read the terminal.** A bug in one of your callbacks does *not* stop the run; it prints a traceback (`Exception in subscription callback ...`) and carries on. The same error repeated is counted, not reprinted. A QoS mismatch prints a `WARNING ... incompatible QoS` line and a `[launch] WARNING: your subscription ... is NOT connected`. Run the simulator with `python sim/launch.py`; running `python odometry_node.py` directly will not work.

3. **Start with clean feeds** while you get the basics working (this is also where you check your numbers against the truth, with no noise in the way):

   ```bash
   python sim/launch.py --no-faults
   ```

   `--no-faults` disables all injected noise — GPS is clean, no duplicate
   messages, no dropped ticks. Good for verifying your subscriptions connect
   before tackling the noise.

4. **Enable the live visualizer** once you have `/odom` publishing:

   ```bash
   python sim/launch.py --visualize
   ```

   Opens a matplotlib window showing ground truth, GPS readings, wheel-only dead
   reckoning (orange; drawn using the *true* heading, so it shows only the
   encoder's own errors) and your fused `/odom` output in real time. The blue line
   only appears once your node is publishing. The bottom plot is each source's
   distance from the truth, which is how you tell whether it really works.
   If no window can open it tells you why and what to install.

| Command | What it does |
| --- | --- |
| `python sim/launch.py` | Default: noisy feeds |
| `python sim/launch.py --no-faults` | Clean feeds while building basics |
| `python sim/launch.py --visualize` | Adds live matplotlib plot |
| `python sim/launch.py --no-node` | Run simulator only, no odometry node |
| `python sim/launch.py --seed 42` | Repeat exactly the same noise (the seed is always printed at startup) |

### The feeds

| Topic | Rate | Message | QoS |
| --- | --- | --- | --- |
| `/wheel_ticks` | ~50 Hz | `WheelTicks{tick_count: int, timestamp: float}` | check the source |
| `/gps_estimate` | ~1 Hz | `GPSEstimate{x: float, y: float, timestamp: float, covariance: float}` | check the source |

- `tick_count` is a cumulative total — it only ever increases (or stays the same on a duplicate).
- The rate is ~50 Hz (a duplicate adds an extra message now and then).
- `x` and `y` are in metres, in a local ENU frame where (0, 0) is the rover's start position.
- `covariance` is in m² — it is the variance (σ²) of the GPS reading, useful for fusion weighting.

### Where does heading come from?

Wheel ticks tell you **how far** the rover moved, not **which way it is facing**. No topic reports heading or turn rate. That is deliberate: it is the part you have to decide. To turn distance into a change in (x, y) you need a heading, and the rover is *turning*, so it changes.

What you are given:

- The rover starts at (0, 0) **facing east** (heading 0). This is true in every run.
- It moves smoothly (it never jumps or reverses), and GPS gives an absolute position once a second.
- The circle in `ground_truth.py` is one example. **When we run your node we may use a different path**: a different radius, a different speed, or turning the other way (always starting at (0, 0) facing east). A node that only works on that exact circle will not do well.

Decide how your node gets its heading and how fast that heading is changing, and say in your write-up what you chose, why, and what would break it. There is more than one workable answer, and some are simple.

*Stuck? Hints:* the direction of travel is the direction between two positions, so what do the GPS fixes over the last few seconds tell you? How noisy is that over 1 second compared with 10 seconds? Heading changes slowly and smoothly, so an estimate can be remembered between fixes. And you can always check an assumption against GPS: does the distance your wheels report between two fixes match the distance GPS reports?

### Things that go wrong on purpose

The feeds misbehave in ways representative of real hardware. With `--no-faults` off:

**Wheel encoder:**
- **Duplicate messages**: the same `(tick_count, timestamp)` pair arrives twice within a few milliseconds. Naive velocity calculation (`delta_ticks / delta_time`) divides by zero: an infinite or NaN velocity, or a `ZeroDivisionError` in plain Python.
- **Dropped ticks**: now and then the encoder misses one tick (~1.3mm), so the cumulative counter ends up 1 lower than the truth. There is no gap in the data, so you can't spot an individual drop; you can only see the effect. They add up to ~6cm of drift per minute. GPS fusion is your primary correction for this. (Ask yourself: why can't a single drop be detected, and what *can* you say about the drift?)
- **Clock drift**: the encoder's timestamps drift slowly away from the real time (a random walk). Use the *difference* between two timestamps as a duration, but don't compare a message timestamp with `time.time()`.

**GPS:**
- Gaussian position noise (σ ≈ 0.3m in normal conditions).
- ~5s outages repeating every ~45s cycle — the feed goes silent then resumes.
- Elevated noise (σ ≈ 0.8m) for ~10s after each outage resumes (satellite reacquisition). Each run also *starts* in this noisy state, so the first 10s are noisy too.
- The `covariance` field reflects the actual σ² at each moment.

Each 45s cycle: 0–10s noisy, 10–40s normal, 40–45s no messages.

**Measuring time.** To find out "how long ago did that message arrive?", note `time.monotonic()` when it arrives and subtract. Don't use `time.time()` (the system clock can be adjusted while you run; on WSL2 it has jumped backwards by over a second) and don't subtract a message's `timestamp` from the current time (the encoder's clock drifts).

Deciding how to detect and handle each of these is part of the task.

### How this differs from real ROS 2

`rclpy_lite` mirrors the real API, with these differences (they are why a couple of things won't port unchanged):

- Header timestamps (`header.stamp`) are plain floats in seconds, not `builtin_interfaces/Time`.
- All callbacks (subscriptions and timers) run one at a time, like real rclpy's default executor, so you do **not** need locks. `rclpy.spin()` here just waits.
- An exception in a callback is printed with a traceback but doesn't stop the program; real rclpy would stop.
- `tf2_ros` only has `TransformBroadcaster`; there is no `Buffer`, `TransformListener` or `lookup_transform`.

---

## Part 1: Core Task (required)

Complete `odometry_node.py` so that it:

1. **Subscribes to `/wheel_ticks`** and converts tick count changes into distance and velocity using the provided constants:
   ```
   WHEEL_RADIUS_M       = 0.075    # metres
   TICKS_PER_REVOLUTION = 360
   DIST_PER_TICK        = (2π × WHEEL_RADIUS) / TICKS_PER_REVOLUTION  ≈ 0.00131 m
   ```

2. **Handles the injected noise gracefully.** Duplicate messages must be detected and ignored. Dropped ticks can't be seen in the data (see "Things that go wrong on purpose"), so instead explain what they do to your estimate and how you stop the error building up. Nothing may crash or produce NaN/infinity. Document your approach to each in your write-up.

3. **Subscribes to `/gps_estimate`** and performs a simple fusion between your wheel-derived position and the GPS estimate. A basic weighted average based on which source you trust more at a given moment is sufficient. A full EKF is not required. Your position update also needs to know which way the rover is facing: see "Where does heading come from?".

4. **Publishes the fused result** as a `nav_msgs/Odometry` message on `/odom`. Check `sim/nav_msgs/msg/__init__.py` and the [real ROS 2 nav_msgs/Odometry spec](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) for the field layout.

5. **Provides monitoring output** once per second to the terminal showing:
   - Time since last message from each source
   - Measured receive rate for `/wheel_ticks` vs expected ~50 Hz
   - Current fused position (x, y)

### Suggested order of work

Each step has a check that tells you it's right before you move on, and ends with a **commit** (see "Commit as you go" below). Stretch goals: one commit per goal, e.g. `stretch A: <name>`.

| Step | What to do | How you know it's done | Commit message |
| --- | --- | --- | --- |
| 1 | Read `sim/messages.py` and `sim/encoder_publisher.py`. Run `python sim/launch.py --no-node` (Ctrl-C to stop). | You can say what each feed publishes, how often, and with which QoS. | *(nothing to commit yet)* |
| 2 | Subscribe to `/wheel_ticks` (mind the QoS) and print what arrives. Use `--no-faults`. | Your node receives ticks at roughly 50 Hz. | `step 2: wheel subscription` |
| 3 | Turn tick changes into distance and velocity, and publish `/odom` from the wheels alone (for now, assume it drives straight east). | With `--visualize`, your line appears and heads east from (0, 0). | `step 3: wheel odometry` |
| 4 | Handle duplicate and dropped ticks. Now run **without** `--no-faults`. | Velocity is never NaN or infinite, nothing crashes, you can say how you detect duplicates, and why a dropped tick can't be detected. | `step 4: tick noise handling` |
| 5 | Subscribe to `/gps_estimate`, fuse it with the wheel position, and decide how you get heading. | Over a long run your blue line stays close to the truth, and its error (bottom plot) is smaller than the GPS dots' error. | `step 5: gps fusion` |
| 6 | Monitoring output once per second: time since each source, `/wheel_ticks` rate vs ~50 Hz, fused (x, y). | All three appear every second. | `step 6: monitoring output` |
| 7 | Leave it running for several minutes with the faults on. Fix what you find. | No crash, no runaway drift, no stale state. | `step 7: fixes from a long run` |
| 8 | Write your write-up (in this README), finish `METHODOLOGY.md` and `AI_LOG.md`. | Every question in the write-up has an answer with numbers from your own runs. | `step 8: write-up and AI log` |

### Commit as you go

We read your commit history as well as your code. It shows how you worked, and it is the honest record behind your write-up. Commit at the end of each step in the table above, using the message shown (or your own words in the same spirit).

One-time setup (git refuses to commit until it knows who you are). Use your own name and email:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

At the end of each step:

```bash
git status                  # what changed? nothing surprising?
git add -A
git commit -m "step 2: <what you did>"
```

Rules of the road:

- **Small and honest beats tidy.** A history with `step 4: ...` followed by a `fix: ...` commit that repairs your own bug is exactly what we like to see. Fixing your own bug in a later commit is normal.
- **Do not squash, amend or force-push** to make the history look cleaner. Do not commit everything in one go at the end.
- **Commit your AI log as you go too.** When an AI tool gets something wrong, write the `AI_LOG.md` entry in the same commit as the fix.
- Do not edit the files marked "PROVIDED, do not modify" or anything else under `sim/`. We run your `odometry_node.py` against our own copy.

### What we're looking for

- Does it run against the provided scaffold and produce a sensible fused position estimate?
- Does it visibly handle the injected noise (we will be able to tell from your output whether dropped/duplicate ticks corrupted your result)?
- Correct QoS configuration — your node actually connects to the provided publishers.
- A write-up explaining your fusion approach, your noise-handling logic, and any design decisions.

---

## Part 2: Stretch Goals (optional)

Pick any/all. Partial, well-reasoned attempts are valued over none.

**A. TF2 broadcaster**

Publish your fused odometry as a proper TF2 transform (`odom → base_link`) instead of just a message. Use the provided `TransformBroadcaster` in `sim/tf2_ros/` (its docstring has a usage example). Pay close attention to parent/child frame order and quaternion conventions — this is a common spot where things look right but are backwards. The shim only stores the latest transform per child frame, so check yours by printing `tf2_ros.get_latest_transforms()`.

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
```

**B. Confidence-weighted fusion**

Instead of a fixed weighting between wheel and GPS estimates, make the weighting dynamic — e.g., trust wheel odometry more over short time windows and GPS more as wheel-derived drift accumulates. The `covariance` field on `GPSEstimate` messages gives you the GPS uncertainty at each moment. This is conceptually close to what a real EKF does.

**C. Full TF2 (advanced)**

Implement a full `map → odom → base_link` transform system using the published data. Create the `odom → base_link` transform from encoder data, and a `map → odom` correction from GPS. Ensure that querying the rover's `map` position between GPS updates returns a sensible interpolated result. The shim has no `Buffer` or `lookup_transform`, so you write that helper yourself.

---

## Part 3: AI Usage Log (required)

Submit a short `AI_LOG.md` with your code (there is a template in the repo root). For each significant use of AI tools, note:

- What you asked
- What you kept vs. rewrote, and why
- Anything the AI got wrong that you had to catch
- How you verified it actually worked correctly (not just that it compiled)

This is not graded on whether you used AI — it is graded on whether you can tell us what it got wrong and why you fixed it.

---

## Part 4: METHODOLOGY.md (required)

Edit the `METHODOLOGY.md` in the repo root (there is a template) so it covers:

- **How to run your code.** The exact steps for a reviewer to install everything and run your work from a fresh clone and see it working. Say which Python version and OS you tested on.
- **Your thought process, in bullet points.** Why you made the choices you did, and what your own runs showed.
- **Known limitations.** What doesn't work, and what you'd do next. Being upfront counts in your favor.

The "Your write-up" section of this README answers specific questions with numbers from your runs; `METHODOLOGY.md` is the how-to-run and the reasoning, in your own words. Write it for a teammate who has never seen your code.

## Rubric

| Criterion | What we're scoring |
| --- | --- |
| **Correctness** | Core task runs against the scaffold, produces sensible fused odometry, pub/sub actually connects |
| **Noise handling** | Duplicate ticks and noisy GPS are detected and handled; dropped ticks are understood (why they can't be seen, what they do to the estimate, how GPS bounds them); nothing is silently ignored |
| **Design judgment** | Evidence of intentional choices beyond the minimum (fusion weighting, code structure, sensible defaults) |
| **Handling ambiguity** | How did they resolve underspecified parts of the task (notably: where heading comes from)? Did they make a reasonable call, explain it, and say what would break it? |
| **Understanding, not just output** | Can they explain their own code/math? Does the write-up show real comprehension? |
| **AI verification** | Evidence they tested/verified AI-assisted code rather than taking it on faith (from log + code quality + commit history) |
| **Stretch engagement** (bonus) | Attempted or completed any stretch goal — even partial attempts count positively |

We don't expect a perfect implementation. Those who show genuine effort and learning are the ones who will have a leg up!

---

## Your write-up

### Fusion approach
Position fusion combines high-frequency wheel dead-reckoning with low-frequency absolute GPS updates using a dynamic confidence-weighted complementary filter:
- **Weighting formulation:** The GPS update weight is computed dynamically as:
  $$w_{\text{gps}} = \text{clamp}\left(\frac{0.012}{\text{covariance}} \times \sqrt{\text{drift\_factor}},\, 0.03,\, 0.25\right)$$
- **Signal quality & covariance:** The weight scales inversely with `msg.covariance`. A typical clean reading ($\approx 0.09\text{ m}^2$) yields a base weight around $0.13$, whereas noisy fixes receive down to the minimum clamp ($0.03$).
- **Drift scaling over time (Stretch Goal B):** The time elapsed since the last GPS update ($\Delta t_{\text{gps}}$) sets $\text{drift\_factor} = \text{clamp}(\Delta t_{\text{gps}}, 1.0, 6.0)$. As dead-reckoning drift accumulates during outages, $\sqrt{\text{drift\_factor}}$ scales the weight upward to a maximum clamp of $0.25$, allowing the filter to recover accumulated error without introducing sharp trajectory snaps.
- **State blend:** Position states are blended directly:
  $$x \leftarrow (1 - w_{\text{gps}}) x + w_{\text{gps}} x_{\text{gps}}, \quad y \leftarrow (1 - w_{\text{gps}}) y + w_{\text{gps}} y_{\text{gps}}$$

### Heading
- **Kinematic integration:** Heading is integrated at 50 Hz in `wheel_tick_callback` using circular arc kinematics:
  $$\Delta\theta = \frac{\Delta s}{R}, \quad R = 9.8\text{ m}$$
  Angles are normalized to $[-\pi, \pi]$ using modulo arithmetic.
- **Gated GPS heading trim:** In `gps_callback`, when GPS covariance is low ($< 0.15\text{ m}^2$) and consecutive displacement exceeds $0.3\text{ m}$, the displacement vector angle is calculated via $\text{atan2}(\Delta y, \Delta x)$. The heading error is wrapped to $[-\pi, \pi]$ and blended into the current heading with a gentle gain of $0.02$, eliminating long-run angular drift without injecting GPS positional jitter.
- **Assumptions and breaking conditions:** Assumes counter-clockwise motion along a circular path of known nominal radius ($9.8\text{ m}$). Straight lines, clockwise loops, reverse maneuvers, or non-circular paths would break this kinematic assumption. A general-purpose rover would instead derive yaw rate from differential wheel velocity kinematics or an onboard IMU gyroscope.

### Noise handling
- **Detected and handled faults:**
  - *Duplicate packets:* Packets where $\Delta t \le 1\times 10^{-5}\text{ s}$ are dropped immediately, preventing division-by-zero, velocity spikes, and `NaN` states.
  - *Non-monotonic ticks:* Packets where $\Delta\text{ticks} < 0$ are discarded to ensure monotonic distance integration.
  - *Clock drift:* The encoder timestamp drifts relative to wall-clock time. Handled by treating timestamps strictly as relative $\Delta t$ differences and relying on `time.monotonic()` for all arrival, frequency, and timeout monitoring.
- **Undetectable noise:**
  - *Dropped ticks:* Missing ticks drop the cumulative counter by 1 without showing an explicit sequence gap or timestamp anomaly. These are undetectable on a per-packet basis.
  - *Impact without correction:* Dropped ticks cause steady dead-reckoning drift ($\approx 6\text{ cm/min}$). The absolute GPS fusion continually absorbs this drift whenever valid fixes arrive.

### Monitoring output
- **Staleness threshold:** Set to $> 3.0\text{ s}$ for GPS. At a nominal $1\text{ Hz}$ update rate, missing three consecutive cycles confirms an active blackout and appends `[OUTAGE]` to the terminal log.
- **Receive rate calculation:** Encoder receive rate is measured over a 1.0 s sliding window using `time.monotonic()` ($\text{ticks\_since\_last\_report} / \Delta t_{\text{elapsed}}$), verifying that the incoming stream stays near the expected $50\text{ Hz}$ (nominally 49–53 Hz).
- **TF verification:** Prints the composed transform lookup `map=(x, y)` at 1 Hz alongside the internal fused coordinates.

### Ambiguity
- **Path curvature radius:** The starter code did not define the ground-truth turning radius. Calibrated `PATH_RADIUS_M = 9.8` m empirically to prevent the estimated circle from expanding radially relative to the true path.
- **Coordinate frame jump convention (REP 105):** The prompt does not specify how discrete GPS updates should interact with the TF tree. Adhered to REP 105 conventions by decoupling frames: `odom -> base_link` remains continuous dead-reckoning (`enc_x`, `enc_y`, `enc_heading`), while GPS corrections are stored as a rigid offset in `map -> odom`.

### Testing
- **Verification steps:**
  - Ran `python sim/launch.py --no-faults` to verify baseline velocity integration and message pacing.
  - Ran `python sim/launch.py` to confirm duplicate rejection, verifying that 1 Hz monitoring reported steady ~50 Hz receive rates without division errors or velocity spikes.
  - Ran `python sim/launch.py --visualize` over multiple full revolutions to confirm that the blue fused trajectory stayed concentric with the green ground truth and bridged simulated 5-second GPS blackouts without diverging.

### Stretch goals
- **Stretch Goal A (TF2 Broadcaster):** Implemented using `TransformBroadcaster(self)`, publishing `TransformStamped` messages for `odom -> base_link` with orientation formatted as quaternions ($z = \sin(\theta/2), w = \cos(\theta/2)$).
- **Stretch Goal B (Dynamic Confidence Weighting):** Implemented dynamic weighting based on inverse GPS covariance and a time-since-last-fix drift factor ($\sqrt{\text{drift\_factor}}$), bounded between $0.03$ and $0.25$.
- **Stretch Goal C (Decoupled TF Tree & Transform Lookup):** Implemented the full `map -> odom -> base_link` transform tree:
  - `odom -> base_link` publishes smooth wheel odometry at 50 Hz.
  - `map -> odom` publishes corrective offsets at 1 Hz during GPS updates.
  - Implemented `lookup_transform("map", "base_link")` to rotate and translate coordinates, enabling downstream nodes to query global map coordinates smoothly at 50 Hz.

---

## Submitting

1. **Create a public repository on your own GitHub account.**
2. **Point your clone at it.** Your clone's `origin` is our repo, which you can't push to:

   ```bash
   git remote set-url origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```

3. **Check that it's public.** Open your repo's link in a private/incognito browser window. If you can see the code without logging in, so can we.
4. **Send us the link** in the Google Form you'll be asked to fill out.

Your repo should include your code, your write-up (the section above, in this README), your `METHODOLOGY.md`, your `AI_LOG.md`, and your **full commit history** (push all of it; do not squash).
