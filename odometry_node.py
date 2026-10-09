"""
Odometry Node — your implementation goes here.

Run via:
    python sim/launch.py
    python sim/launch.py --visualize      # with live plot
    python sim/launch.py --no-faults      # clean feeds while getting started

This file uses rclpy_lite, a lightweight simulator shim that mirrors the real
rclpy (ROS 2 Python) API. The class names, method signatures and message field
names match real ROS 2, and the import paths differ:

    Real ROS 2:     import rclpy / from rclpy.node import Node
    This shim:      import rclpy_lite as rclpy / from rclpy_lite.node import Node

(The README lists the few small differences, e.g. header stamps are plain floats.)

Two things that make life easier here:
  - Callbacks never run at the same time as each other (real rclpy does the same
    with its default executor), so you do not need locks around your state.
  - If a callback raises an exception you will see a traceback in the terminal.
    The run keeps going, so read the terminal, don't just look for a crash.
"""

import sys

if __name__ == "__main__":
    # This file is loaded BY the simulator; running it directly can't work.
    # (Delete this guard if you ever port the node to a real ROS 2 install.)
    sys.exit("Don't run this file directly. Start everything with:\n\n    python sim/launch.py\n")

import math
import time

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message types provided by the simulator
from sim.messages import WheelTicks, GPSEstimate

# Standard ROS message types (mirrored by the shim)
# Also imports for stretch A
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster

# --------------------------------------------------------------------------
# Constants  (do not change — these match the real rover's encoder setup)
# --------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.075          # metres
TICKS_PER_REVOLUTION = 360
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION  # ~0.00131 m

# Radius of curvature for ground-truth path (metres)
# Tweaking for accuracy - blue line staying outside so reducing
PATH_RADIUS_M = 9.8


class OdometryNode(Node):
    """
    Fuses wheel encoder ticks with GPS estimates to produce odometry.

    Subscribes:
        /wheel_ticks   (sim.messages.WheelTicks)   ~50 Hz, BEST_EFFORT
        /gps_estimate  (sim.messages.GPSEstimate)   ~1 Hz,  BEST_EFFORT

    Publishes:
        /odom          (nav_msgs/Odometry)          matches input rate
    """

    def __init__(self):
        super().__init__("odometry_node")

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        wheel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        
        self.tick_sub = self.create_subscription(
            WheelTicks, "/wheel_ticks", self.wheel_tick_callback, wheel_qos
        )

        # TODO: Create a subscriber for /wheel_ticks.
        #
        # IMPORTANT: Check what QoS the encoder publisher uses (see
        # sim/encoder_publisher.py). A mismatch means you will receive
        # zero messages — with no error or warning, just silence.
        # This is intentional and mirrors real ROS 2 behaviour.
        #
        # Hint: the QoS profile you need looks like:
        #   QoSProfile(reliability=ReliabilityPolicy.???, depth=10)
        #
        # self.tick_sub = self.create_subscription(
        #     WheelTicks, "/wheel_ticks", self.wheel_tick_callback, ???
        # )

        self.gps_sub = self.create_subscription(
            GPSEstimate, "/gps_estimate", self.gps_callback, wheel_qos
        )

        # TODO: Create a subscriber for /gps_estimate (same QoS considerations).
        #
        # self.gps_sub = self.create_subscription(
        #     GPSEstimate, "/gps_estimate", self.gps_callback, ???
        # )

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        # TODO: Create a publisher for /odom.
        #
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        # Stretch Goal A: TF2 Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------------
        # Timer — 1 Hz monitoring output
        # ------------------------------------------------------------------
        # TODO: Create a timer that calls self.monitoring_callback once per second.
        #
        self.monitor_timer = self.create_timer(1.0, self.monitoring_callback)

        # ------------------------------------------------------------------
        # State  — add whatever you need
        # ------------------------------------------------------------------
        self.x: float = 0.0                  # fused position, metres (east)
        self.y: float = 0.0                  # fused position, metres (north)
        self.heading: float = 0.0            # radians. The rover starts facing EAST (0 rad);
                                             # x is east, y is north, counter-clockwise is positive
        # Stretch Goal C
        self.enc_x: float = 0.0
        self.enc_y: float = 0.0
        self.enc_heading: float = 0.0

        self.map_to_odom_x: float = 0.0
        self.map_to_odom_y: float = 0.0
        self.map_to_odom_yaw: float = 0.0

        #----

        self.linear_velocity: float = 0.0
        self.last_tick_count: int | None = None
        self.last_tick_time: float | None = None

        self.last_wheel_time: float | None = None
        self.last_gps_time: float | None = None
        self.last_gps_arrival: float | None = None

        self.prev_gps_x: float | None = None
        self.prev_gps_y: float | None = None
        
        self.wheel_msg_count: int = 0
        self.start_time: float = time.monotonic()   # use time.monotonic() to measure durations

        self.ticks_since_last_report: int = 0
        self.last_report_monotonic: float = time.monotonic()

    # -----------------------------------------------------------------------
    # Wheel encoder callback
    # -----------------------------------------------------------------------

    def wheel_tick_callback(self, msg: WheelTicks) -> None:
        """
        Called each time a wheel tick message arrives (~50 Hz).

        Convert the change in tick count to a distance, then update your
        position estimate and publish odometry.

        Args:
            msg.tick_count   Cumulative encoder ticks (monotonically increases).
            msg.timestamp    Wall-clock seconds when the reading was taken.
                             Subject to slow clock drift — see README.

        Noise to handle:
            DUPLICATE messages   The same (tick_count, timestamp) pair is
                                 occasionally re-sent within a few ms.
                                 If you compute velocity as delta_ticks / delta_time
                                 and delta_time is 0, you get a NaN, an infinite
                                 velocity, or a ZeroDivisionError. Check for this.

            DROPPED ticks        Now and then the encoder misses one tick (~1.3mm)
                                 and the cumulative counter is simply 1 lower than
                                 it should be. The data never shows a gap, so you
                                 cannot spot an individual drop. The error just
                                 adds up (~6cm per minute), and GPS fusion is your
                                 correction. (Think about why you can't see it.)

            CLOCK DRIFT          msg.timestamp wanders slowly away from the real
                                 time. Use differences between timestamps as
                                 durations; never compare them with time.time().

        Useful constants:
            DIST_PER_TICK   metres per encoder tick (~0.00131 m)
        """

        self.last_wheel_time = time.monotonic()
        self.wheel_msg_count += 1
        self.ticks_since_last_report += 1
        #print(f"[wheel_ticks] count={msg.tick_count} stamp={msg.timestamp:.3f}")

        # 1. Handle first message (initialise last_tick_count / last_tick_time)
        if self.last_tick_count is None or self.last_tick_time is None:
            self.last_tick_count = msg.tick_count
            self.last_tick_time = msg.timestamp
            return

        # 2. Compute delta_ticks and delta_time
        delta_ticks = msg.tick_count - self.last_tick_count
        delta_time = msg.timestamp - self.last_tick_time

        # 3. Guard against duplicate messages and non-positive time intervals
        # A duplicate message has identical timestamp or tick count received in ~0s
        if delta_time <= 1e-5:
            # Drop duplicate / zero-duration packet to prevent ZeroDivisionError or NaN
            #print(f"[REJECTED DUP] delta_t={delta_time:.6f}s delta_ticks={delta_ticks}")
            return

        if delta_ticks < 0:
            # Ticks are strictly monotonic; ignore out-of-order anomalies
            #print(f"[REJECTED REVERSAL] delta_ticks={delta_ticks}")
            return
        
        self.last_tick_count = msg.tick_count
        self.last_tick_time = msg.timestamp

        # 4. Convert ticks to distance: distance = delta_ticks * DIST_PER_TICK
        distance = delta_ticks * DIST_PER_TICK
        if delta_time > 0:
            self.linear_velocity = distance / delta_time

        #if self.wheel_msg_count % 50 == 0:
        #    print(f"[OK] v={self.linear_velocity:.3f} m/s | dt={delta_time:.4f}s | ticks={delta_ticks}")

        # 5. Continuous kinematic heading update (counter-clockwise arc)
        # delta_theta = arc_length / radius
        self.heading += distance / PATH_RADIUS_M
        self.heading = (self.heading + math.pi) % (2.0 * math.pi) - math.pi

        # 6. Update self.x and self.y
        self.x += distance * math.cos(self.heading)
        self.y += distance * math.sin(self.heading)

        self.enc_heading += distance / PATH_RADIUS_M
        self.enc_heading = (self.enc_heading + math.pi) % (2.0 * math.pi) - math.pi
        self.enc_x += distance * math.cos(self.enc_heading)
        self.enc_y += distance * math.sin(self.enc_heading)

        # 7. Call self.publish_odometry()
        self.publish_odometry(stamp=msg.timestamp)

    # -----------------------------------------------------------------------
    # GPS callback
    # -----------------------------------------------------------------------

    def gps_callback(self, msg: GPSEstimate) -> None:
        """
        Called each time a GPS estimate arrives (~1 Hz, with occasional outages).

        Fuse the GPS absolute position with your wheel-derived position.
        A weighted average is sufficient — no EKF required.

        Args:
            msg.x           GPS east position (metres from start).
            msg.y           GPS north position (metres from start).
            msg.timestamp   Seconds since epoch when the reading was taken.
            msg.covariance  Uncertainty (m²). Higher = less reliable.
                            Use this to adjust how much you trust the GPS
                            reading relative to your wheel estimate.

        Design question:
            GPS is noisy but absolute (no accumulated drift).
            Wheel odometry drifts but is precise over short distances.
            How do you weight them? Should the weighting change over time?
            (That's Stretch Goal B — but even a fixed weighting is fine here.)
        """

        # Record arrival using monotonic clock for timeout/monitoring checks
        self.last_gps_arrival = time.monotonic()

        if msg.covariance < 0.15 and self.prev_gps_x is not None:
            dx = msg.x - self.prev_gps_x
            dy = msg.y - self.prev_gps_y
            disp = math.hypot(dx, dy)
            if disp > 0.3:
                gps_heading = math.atan2(dy, dx)
                diff = (gps_heading - self.heading + math.pi) % (2.0 * math.pi) - math.pi
                self.heading = (self.heading + 0.02 * diff + math.pi) % (2.0 * math.pi) - math.pi

        self.prev_gps_x = msg.x
        self.prev_gps_y = msg.y

        # 1. Compute a weight w_gps based on msg.covariance (lower cov = higher trust).
        dt_gps = (msg.timestamp - self.last_gps_time) if self.last_gps_time is not None else 1.0
        drift_factor = min(max(dt_gps, 1.0), 6.0)
        # cov ~ 0.09 -> w ~ 0.12
        w_gps = min(max((0.012 / msg.covariance) * math.sqrt(drift_factor), 0.03), 0.25)

        # 2. Blend: self.x = (1 - w_gps) * self.x + w_gps * msg.x
        #           self.y = (1 - w_gps) * self.y + w_gps * msg.y
        self.x = (1.0 - w_gps) * self.x + w_gps * msg.x
        self.y = (1.0 - w_gps) * self.y + w_gps * msg.y

        # Stretch Goal C:
        self.map_to_odom_yaw = (self.heading - self.enc_heading + math.pi) % (2.0 * math.pi) - math.pi
        c_yaw = math.cos(self.map_to_odom_yaw)
        s_yaw = math.sin(self.map_to_odom_yaw)
        
        rot_x = self.enc_x * c_yaw - self.enc_y * s_yaw
        rot_y = self.enc_x * s_yaw + self.enc_y * c_yaw
        self.map_to_odom_x = self.x - rot_x
        self.map_to_odom_y = self.y - rot_y

        self.broadcast_map_to_odom(msg.timestamp)
        
        # 3. Update self.last_gps_time = msg.timestamp
        self.last_gps_time = msg.timestamp

    # -----------------------------------------------------------------------
    # Odometry publisher
    # -----------------------------------------------------------------------

    def publish_odometry(self, stamp: float | None = None) -> None:
        """
        Build a nav_msgs/Odometry message from the current state and publish it.

        The Odometry message layout (real ROS 2 spec, mirrored by the shim):
            msg.header.stamp         float, seconds since epoch
            msg.header.frame_id      str,   "odom"
            msg.child_frame_id       str,   "base_link"
            msg.pose.pose.position.x float, metres east
            msg.pose.pose.position.y float, metres north
            msg.pose.pose.orientation  Quaternion(x, y, z, w)
                For a 2D robot rotating around z: yaw → quaternion:
                    z = sin(heading / 2)
                    w = cos(heading / 2)
                    x = 0, y = 0
            msg.twist.twist.linear.x float, forward speed (m/s)
            msg.twist.twist.angular.z float, yaw rate (rad/s)

        Look up the real nav_msgs/Odometry definition for the full field list.
        Not every field is required — position is the minimum.
        """


        msg = Odometry()
        msg.header.stamp = stamp if stamp is not None else time.time()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.heading / 2.0),
            w=math.cos(self.heading / 2.0),
        )

        msg.twist.twist.linear.x = self.linear_velocity
        msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(msg)

        # Stretch Goal A/C:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"          # Parent frame
        t.child_frame_id = "base_link"      # Child frame

        t.transform.translation.x = self.enc_x
        t.transform.translation.y = self.enc_y
        t.transform.translation.z = 0.0

        t.transform.rotation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.enc_heading / 2.0),
            w=math.cos(self.enc_heading / 2.0),
        )
        
        self.tf_broadcaster.sendTransform(t)
        

    # -----------------------------------------------------------------------
    # Monitoring callback  (1 Hz)
    # -----------------------------------------------------------------------

    def monitoring_callback(self) -> None:
        """
        Print a one-line status update to the terminal once per second.

        Required output (see README rubric):
            - Seconds since last /wheel_ticks message
            - Seconds since last /gps_estimate message
            - Measured /wheel_ticks receive rate vs expected ~50 Hz
            - Current fused position (x, y)

        Tip: record time.monotonic() each time a message ARRIVES and subtract.
        Don't use msg.timestamp for this: the encoder's clock drifts.

        Example format (feel free to change the layout):
            [odom] pos=(1.23, 0.45)m  enc=0.02s ago @49.8Hz  gps=0.91s ago
        """
        now = time.monotonic()

        # Time elapsed since the last incoming message from each sensor
        dt_wheel = (now - self.last_wheel_time) if self.last_wheel_time is not None else float("nan")
        dt_gps = (now - self.last_gps_arrival) if self.last_gps_arrival is not None else float("nan")

        # Measured wheel message frequency over the last 1.0 second
        elapsed = now - self.last_report_monotonic
        measured_hz = (self.ticks_since_last_report / elapsed) if elapsed > 0 else 0.0

        # Reset counters for the next window
        self.ticks_since_last_report = 0
        self.last_report_monotonic = now

        # Add outage flag if GPS drops out (> 3.0s)
        gps_status = f"{dt_gps:4.2f}s ago" if dt_gps < 3.0 else f"{dt_gps:4.2f}s ago [OUTAGE]"

        # verification for Stretch Goal A:
        import tf2_ros
        transforms = tf2_ros.get_latest_transforms()
        tf_status = f"tf_active({transforms[0].child_frame_id})" if transforms else "tf_none"

        # Verify Stretch Goal C:
        map_pose = self.lookup_transform("map", "base_link")
        map_str = f"map=({map_pose[0]:.2f}, {map_pose[1]:.2f})" if map_pose else "map=pending"

        print(
            f"[odom] pos=({self.x:5.2f}, {self.y:5.2f})m | "
            f"enc={dt_wheel:5.3f}s ago @ {measured_hz:4.1f}Hz (exp ~50Hz) | "
            f"gps={gps_status} | {map_str}"
        )

    def broadcast_map_to_odom(self, stamp: float) -> None:
        t_map = TransformStamped()
        t_map.header.stamp = stamp
        t_map.header.frame_id = "map"
        t_map.child_frame_id = "odom"

        t_map.transform.translation.x = self.map_to_odom_x
        t_map.transform.translation.y = self.map_to_odom_y
        t_map.transform.translation.z = 0.0

        t_map.transform.rotation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.map_to_odom_yaw / 2.0),
            w=math.cos(self.map_to_odom_yaw / 2.0),
        )
        self.tf_broadcaster.sendTransform(t_map)

    def lookup_transform(self, target_frame: str, source_frame: str):
        import tf2_ros
        transforms = {t.child_frame_id: t for t in tf2_ros.get_latest_transforms()}

        if target_frame == "map" and source_frame == "base_link":
            if "odom" not in transforms or "base_link" not in transforms:
                return None
            t_m2o = transforms["odom"]
            t_o2b = transforms["base_link"]

            # yaw = 2 * atan2(z, w)
            yaw_m2o = 2.0 * math.atan2(t_m2o.transform.rotation.z, t_m2o.transform.rotation.w)
            yaw_o2b = 2.0 * math.atan2(t_o2b.transform.rotation.z, t_o2b.transform.rotation.w)

            yaw_composed = (yaw_m2o + yaw_o2b + math.pi) % (2.0 * math.pi) - math.pi

            x_b = t_o2b.transform.translation.x
            y_b = t_o2b.transform.translation.y
            x_composed = t_m2o.transform.translation.x + (x_b * math.cos(yaw_m2o) - y_b * math.sin(yaw_m2o))
            y_composed = t_m2o.transform.translation.y + (x_b * math.sin(yaw_m2o) + y_b * math.cos(yaw_m2o))

            return x_composed, y_composed, yaw_composed
        return None

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
