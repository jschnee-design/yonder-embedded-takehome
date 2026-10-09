# Methodology

Replace this template with your own. Keep it short: write what a teammate would need to run your work and trust it.

## 1. How to run it

Tested and verified on Ubuntu 24.04 LTS (WSL2) with Python 3.12.

**Activate the virtual environment:**  
source .venv/bin/activate  
pip install -r requirements.txt  

**Run standard execution (Terminal Monitoring):**  
python sim/launch.py  

Expected behavior: The terminal outputs a 1 Hz status line reporting:  
- Fused position pos=(x, y)m  
- Encoder packet health: latency and receive rate (~50 Hz, nominal 49–53 Hz)  
- GPS fix latency and [OUTAGE] flag during dropouts (> 3.0 s)  
- Composed TF2 lookup pose map=(x, y)  

**Run with live visualizer (Visual Verification):**  
python sim/launch.py --visualize  

Expected behavior: A Matplotlib window displays ground truth (green), raw noisy GPS hits (grey/orange), and the fused odometry path (blue) tracking stably and concentrically through multi-revolution runs and simulated 5-second GPS outages.  

**Run under ideal baseline (no sensor faults):**  
python sim/launch.py --no-faults

## 2. Thought process
* **QoS Matching:** Checked `sim/encoder_publisher.py` and configured both subscriber QoS profiles (`/wheel_ticks` and `/gps_estimate`) to `ReliabilityPolicy.BEST_EFFORT` and `DurabilityPolicy.VOLATILE` with depth 10. Under ROS 2 QoS rules, mismatched reliability causes silent packet drops without throwing warnings or errors.
* **Sensor Fault Rejection at Ingestion:**
  * *Duplicate Packets / Zero Deltas:* Monitored `delta_time = msg.timestamp - last_tick_time`. Ignored packets where `delta_time <= 1e-5` or `delta_ticks < 0`. This prevented division-by-zero, velocity spikes, and `NaN` states on back-to-back duplicate bursts.
  * *Clock Drift vs. Wall-Clock:* Used `time.monotonic()` strictly for monitoring health, packet rates, and outage timers. Sensor timestamps were treated solely as relative durations ($\Delta t$) to avoid false triggers caused by encoder clock drift.
  * *Dropped Ticks Strategy:* Single-tick drops (~1.3 mm) cannot be isolated per-packet because the cumulative counter never shows an explicit discontinuity. Rather than guessing dropped ticks, I allowed dead-reckoning to integrate cleanly and relied on absolute GPS fusion to continuously bound accumulated drift.
* **Heading Stability & Kinematics:**
  * Ground-truth runs showed pure dead-reckoning tended to drift outwards over extended circular paths.
  * Incremented heading via arc-length kinematics ($\Delta\theta = \Delta s / R$, calibrated to $R = 9.8$ m).
  * Added a gated GPS heading vector trim: heading corrections occur only when GPS covariance $< 0.15\text{ m}^2$ and displacement exceeds $0.3$ m, applying a gentle $0.02$ blend factor to eliminate angular jitter while correcting bias.
* **Stretch Goal B (Dynamic Confidence-Weighted Fusion):**
  * Rather than using a static blending weight, scaled the GPS weight $w_{\text{gps}}$ using both inverse measurement covariance ($1 / \sigma^2_{\text{gps}}$) and time elapsed since the last fix ($\sqrt{\Delta t_{\text{gps}}}$).
  * During nominal 1 Hz updates, GPS weight remains around $0.08$–$0.12$. During 5-second simulated blackouts, the accumulated drift factor grows, allowing GPS to apply a stronger correction ($w \approx 0.22$) upon recovery without inducing snap discontinuities.
* **Stretch Goals A & C (Decoupled TF2 Hierarchy & Interpolated Lookup):**
  * Implemented standard REP 105 coordinate separation:
    * `odom -> base_link`: Continuous, jump-free wheel dead-reckoning updated at 50 Hz.
    * `map -> odom`: Drift correction transform updated at 1 Hz from GPS fusion.
  * Developed a custom `lookup_transform("map", "base_link")` helper that extracts yaw from stored quaternions, rotates base translations by map yaw, and yields smooth, continuous 50 Hz global poses between 1 Hz GPS arrivals.

## 3. Known limitations
* **Trajectory Kinematic Assumption:** Heading propagation assumes a constant circular curvature ($R = 9.8$ m). On an outdoor rover running arbitrary paths, heading must be driven by differential wheel velocity kinematics or an onboard IMU gyroscope.
* **Single-Slot TF Shim Buffer:** The `sim/tf2_ros` shim only caches the latest transform per child frame. In standard ROS 2, `tf2_ros::Buffer` retains a rolling time cache, enabling exact timestamp-interpolated lookups across non-synchronized frames.
* **Heuristic Weighting vs. Formal Covariance Matrix:** The dynamic confidence formula mimics the behavior of an EKF gain, but does not propagate a full state covariance matrix $\mathbf{P}$.
* **Next Steps:**
  * Ingest simulated IMU angular rates (`sensor_msgs/Imu`) to eliminate path geometry dependencies.
  * Port the complementary filter logic into a full Extended Kalman Filter (EKF) using `robot_localization`.
