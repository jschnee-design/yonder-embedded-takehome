# AI Usage Log

## 1. <Initial Understanding>

**What I asked:**
I gave it the README and had it breakdown how ROS 2 works

**What I kept vs. rewrote, and why:**
N/A no code generation

**What the AI got wrong that I had to catch:**
N/A

**How I verified it ran correctly (not just that it compiled):**
Went through the files outlined in the Familiarisation section of the README

## 2. <Heading Drift Correction & Long-Run Fusion Stability>

**What I asked:**
How to prevent the circular trajectory estimate from drifting outwards over multi-revolution runs under encoder tick noise and GPS outages.

**What I kept vs. rewrote, and why:**
Kept: Arc-length kinematic integration (`heading += distance / PATH_RADIUS_M`) and complementary blending of $(x, y)$ positions with GPS fixes.
Rewrote: The heading trim mechanism. Raw GPS points have significant jitter ($\sigma \approx 0.3\text{ m}$), so taking heading directly from consecutive GPS fixes caused severe angular noise. I added strict gates: requiring low GPS covariance ($< 0.15\text{ m}^2$), minimum displacement ($> 0.3\text{ m}$), and wrapping angular differences via `atan2` with a gentle blend factor ($0.02$) to prevent heading jumps.

**What the AI got wrong that I had to catch:**
The AI initially treated GPS updates as an instant snap or applied a fixed weighting coefficient regardless of GPS noise spikes, which distorted the circular path whenever a noisy fix arrived. Additionally, the default path radius needed tuning to $9.8\text{ m}$ to align with the ground-truth trajectory rather than drifting radially outward.

**How I verified it ran correctly (not just that it compiled):**
Ran `python sim/launch.py --visualize` across many MANY full revolutions. Verified that the blue fused trajectory maintained concentric alignment with the ground-truth circular path and stayed completely stable through simulated 5-second GPS blackouts.

## 3. <Stretch Goal A & B: TF2 Broadcaster and Dynamic Confidence Weighting>

**What I asked:**
How to broadcast `odom -> base_link` transforms using the shim's `TransformBroadcaster` (Stretch A) and dynamically weight GPS fusion based on sensor covariance and accumulated dead-reckoning drift (Stretch B).

**What I kept vs. rewrote, and why:**
Kept: Broadcasting `TransformStamped` with quaternion rotations ($z = \sin(\theta/2), w = \cos(\theta/2)$) and querying `tf2_ros.get_latest_transforms()` to verify active state.
Rewrote: The weighting equation for Stretch Goal B. The initial suggestion only scaled by $1 / \text{covariance}$. I incorporated elapsed time since the last GPS fix ($dt_{\text{gps}}$), scaling confidence by $\sqrt{\Delta t}$ to model random-walk wheel drift accumulation and clamping the gain ($0.03 \le w \le 0.25$) to prevent noise shocks upon GPS reacquisition.

**What the AI got wrong that I had to catch:**
In the initial refactor for Stretch B, the AI drafted the `drift_factor` computation but omitted multiplying it into the actual `w_gps` assignment, leaving the weight static. I caught the omission and ensured `math.sqrt(drift_factor)` scaled the base covariance trust directly.

**How I verified it ran correctly (not just that it compiled):**
Stretch A: Confirmed `tf_active(base_link)` was printed, indicating successful storage in `sim/tf2_ros`.
Stretch B: Observed tracking during an outage (`gps=5.02s ago [OUTAGE]`); upon the next fix, the node smoothly absorbed the accumulated drift without trajectory snapping or divergence.

## 4. <Stretch Goal C: Decoupled TF2 Hierarchy (`map -> odom -> base_link`)>

**What I asked:**
How to implement a full `map -> odom -> base_link` coordinate frame system and a custom `lookup_transform` helper to interpolate the rover's global map position between 1 Hz GPS updates without a full TF2 buffer.

**What I kept vs. rewrote, and why:**
Kept: The mathematical decomposition where `odom -> base_link` tracks continuous, smooth dead-reckoning from wheel encoders, while `map -> odom` holds the offset correction calculated during GPS updates.
Rewrote: Frame lookup composition. Standard ROS transforms require rotating child translations by parent yaw. I wrote the composition math explicitly: extracted yaw from the stored quaternions, rotated $(x_{\text{base}}, y_{\text{base}})$ by $\theta_{\text{map}\to\text{odom}}$, and added the translational offsets.

**What the AI got wrong that I had to catch:**
1. In the initial draft, `broadcast_map_to_odom` and `lookup_transform` were declared at file level with 0 indentation rather than inside the `OdometryNode` class.
2. In `publish_odometry`, the transform fields were populated but `self.tf_broadcaster.sendTransform(t)` was omitted, leaving `lookup_transform` stuck on `map=pending`.
3. An indentation/formatting issue during pasting caused `self.wheel_msg_count` to be misplaced, triggering an `AttributeError` at startup.

**How I verified it ran correctly (not just that it compiled):**
Ran `python sim/launch.py` and monitored the terminal output. Confirmed that `map=(x, y)` was populated immediately upon the first transform publication, exactly matched the fused `pos=(x, y)` values, and updated smoothly at 50 Hz between 1 Hz GPS arrivals.

## Note: Most of this document was generated with the help of AI to give accurate responses :)