#!/usr/bin/env python3
"""Manual and 3-D waypoint controller for the simulated BlueROV2 Heavy.

The node closes the loop on Gazebo odometry and sends the eight ESC commands
as ``actuator_msgs/Actuators``.  Depth is positive down, while Gazebo uses ENU
(``z`` positive up).
"""

import math
import select
import sys
import termios
import tty

import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# Mission waypoints: (latitude [deg], longitude [deg], depth [m]).
# They are intentionally close to the world origin so a test mission is quick.
WAYPOINTS_GPS_DEPTH = [
    (36.509513, -6.262906, 5.0),
    (36.509649, -6.262906, 10.0),
    (36.509649, -6.263074, 5.0),
]

HELP = """
BlueROV2 Heavy - controllo UUV
  W/S       aumenta/diminuisce la velocita' avanti
  A/D       aumenta la rotazione a sinistra/destra
  J/L       traslazione laterale sinistra/destra
  R/F       diminuisce/aumenta la profondita' obiettivo
  SPAZIO/X  arresta il moto orizzontale e mantiene la profondita'
  M         commuta MANUALE / WAYPOINT
  N         salta il waypoint corrente
  H         mostra questo aiuto
  CTRL+C    arresta i thruster ed esce
"""


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class UuvController(Node):
    """Velocity/depth controller with manual and autonomous command sources."""

    def __init__(self):
        super().__init__('uuv_controller')

        # World geodetic origin (ocean_world.sdf).
        self.declare_parameter('latitude_origin', 36.509513)
        self.declare_parameter('longitude_origin', -6.263074)
        self.declare_parameter('start_mode', 'manual')
        self.declare_parameter('max_depth', 80.0)
        self.declare_parameter('max_forward_speed', 1.2)
        self.declare_parameter('max_lateral_speed', 0.6)
        self.declare_parameter('max_vertical_speed', 0.35)
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('waypoint_radius', 1.5)
        self.declare_parameter('depth_tolerance', 0.4)

        self.lat0 = float(self.get_parameter('latitude_origin').value)
        self.lon0 = float(self.get_parameter('longitude_origin').value)
        requested_mode = str(self.get_parameter('start_mode').value).lower()
        self.mode = 'waypoint' if requested_mode == 'waypoint' else 'manual'
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.max_forward_speed = float(
            self.get_parameter('max_forward_speed').value)
        self.max_lateral_speed = float(
            self.get_parameter('max_lateral_speed').value)
        self.max_vertical_speed = float(
            self.get_parameter('max_vertical_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.waypoint_radius = float(
            self.get_parameter('waypoint_radius').value)
        self.depth_tolerance = float(
            self.get_parameter('depth_tolerance').value)

        self.waypoints = [
            (*self.gps_to_enu(lat, lon), depth)
            for lat, lon, depth in WAYPOINTS_GPS_DEPTH
        ]
        self.waypoint_index = 0
        self.mission_complete = False

        # Estimated state from Gazebo odometry.
        self.odom_received = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.yaw_rate = 0.0

        # Manual targets are velocities, except for depth (closed-loop hold).
        self.manual_u = 0.0
        self.manual_v = 0.0
        self.manual_yaw_rate = 0.0
        self.target_depth = None
        self.depth_integral = 0.0

        # Normalized motor state. Slew limiting models the ESC / motor response
        # and avoids unrealistic instantaneous changes in thrust.
        self.motor_command = [0.0] * 8
        self.motor_slew_rate = 1.5  # normalized command per second
        self.dt = 0.05

        self.actuator_pub = self.create_publisher(
            Actuators,
            '/uuv_bluerov2_heavy/command/motor_speed',
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/uuv_bluerov2_heavy/odometry',
            self.odom_callback,
            10,
        )
        # A Point can inject a local ENU waypoint: x=east, y=north,
        # z=positive depth. This mirrors the vessel's /target_waypoint usage.
        self.target_sub = self.create_subscription(
            Point,
            '/uuv/target_waypoint',
            self.target_callback,
            10,
        )
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.terminal_settings = None
        if sys.stdin.isatty():
            self.terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self.get_logger().warning(
                'Terminale non interattivo: tastiera disabilitata; '
                'restano disponibili waypoint e parametri ROS.')

        self.get_logger().info(
            f'UUV controller avviato in modalita {self.mode.upper()}')
        for index, (east, north, depth) in enumerate(self.waypoints):
            self.get_logger().info(
                f'WP{index}: E={east:.1f} m, N={north:.1f} m, '
                f'profondita={depth:.1f} m')
        if self.terminal_settings is not None:
            print(HELP)

    def gps_to_enu(self, latitude, longitude):
        """Small-area WGS84 approximation, matching the vessel controller."""
        radius = 6378137.0
        east = radius * math.radians(longitude - self.lon0) * math.cos(
            math.radians(self.lat0))
        north = radius * math.radians(latitude - self.lat0)
        return east, north

    @staticmethod
    def yaw_from_quaternion(q):
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z
        self.yaw = self.yaw_from_quaternion(pose.orientation)
        self.u = twist.linear.x
        self.v = twist.linear.y
        self.w = twist.linear.z
        self.yaw_rate = twist.angular.z
        self.odom_received = True

        if self.target_depth is None:
            self.target_depth = clamp(-self.z, 0.5, self.max_depth)

    def target_callback(self, msg):
        """Replace the mission with one externally supplied 3-D waypoint."""
        depth = clamp(float(msg.z), 0.5, self.max_depth)
        self.waypoints = [(float(msg.x), float(msg.y), depth)]
        self.waypoint_index = 0
        self.mission_complete = False
        self.mode = 'waypoint'
        self.depth_integral = 0.0
        self.get_logger().info(
            f'Nuovo waypoint esterno: E={msg.x:.1f}, N={msg.y:.1f}, '
            f'profondita={depth:.1f} m')

    def read_key(self):
        if self.terminal_settings is None:
            return ''
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1) if ready else ''

    def handle_keyboard(self):
        key = self.read_key().lower()
        if not key:
            return

        if key == 'm':
            self.mode = 'waypoint' if self.mode == 'manual' else 'manual'
            self.depth_integral = 0.0
            self.get_logger().info(f'Modalita {self.mode.upper()}')
            return
        if key == 'h':
            print(HELP)
            return
        if key == 'n' and self.mode == 'waypoint':
            self.waypoint_index += 1
            self.depth_integral = 0.0
            self.get_logger().info('Waypoint saltato')
            return
        if self.mode != 'manual':
            return

        if key == 'w':
            self.manual_u = clamp(
                self.manual_u + 0.15, -0.5, self.max_forward_speed)
        elif key == 's':
            self.manual_u = clamp(
                self.manual_u - 0.15, -0.5, self.max_forward_speed)
        elif key == 'a':
            self.manual_yaw_rate = clamp(
                self.manual_yaw_rate + 0.10,
                -self.max_yaw_rate,
                self.max_yaw_rate,
            )
        elif key == 'd':
            self.manual_yaw_rate = clamp(
                self.manual_yaw_rate - 0.10,
                -self.max_yaw_rate,
                self.max_yaw_rate,
            )
        elif key == 'j':
            self.manual_v = clamp(
                self.manual_v + 0.10,
                -self.max_lateral_speed,
                self.max_lateral_speed,
            )
        elif key == 'l':
            self.manual_v = clamp(
                self.manual_v - 0.10,
                -self.max_lateral_speed,
                self.max_lateral_speed,
            )
        elif key == 'r' and self.target_depth is not None:
            self.target_depth = clamp(
                self.target_depth - 0.5, 0.5, self.max_depth)
            self.depth_integral = 0.0
        elif key == 'f' and self.target_depth is not None:
            self.target_depth = clamp(
                self.target_depth + 0.5, 0.5, self.max_depth)
            self.depth_integral = 0.0
        elif key in (' ', 'x'):
            self.manual_u = 0.0
            self.manual_v = 0.0
            self.manual_yaw_rate = 0.0

    def waypoint_targets(self):
        """Return desired surge, sway, yaw rate and depth."""
        if self.waypoint_index >= len(self.waypoints):
            if not self.mission_complete:
                self.mission_complete = True
                self.get_logger().info('Missione UUV completata')
            return 0.0, 0.0, 0.0, self.target_depth

        target_x, target_y, target_depth = self.waypoints[self.waypoint_index]
        dx = target_x - self.x
        dy = target_y - self.y
        horizontal_distance = math.hypot(dx, dy)
        depth_error = target_depth - (-self.z)

        if (horizontal_distance <= self.waypoint_radius and
                abs(depth_error) <= self.depth_tolerance):
            self.get_logger().info(
                f'WP{self.waypoint_index} raggiunto: distanza='
                f'{horizontal_distance:.2f} m, errore profondita='
                f'{depth_error:.2f} m')
            self.waypoint_index += 1
            self.depth_integral = 0.0
            return 0.0, 0.0, 0.0, target_depth

        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_angle(desired_yaw - self.yaw)
        desired_yaw_rate = clamp(
            1.2 * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)

        # Slow down while turning and on final approach. This avoids the
        # point-mass behaviour of an unrealistically aggressive controller.
        alignment = max(0.0, math.cos(yaw_error))
        desired_u = min(
            self.max_forward_speed,
            0.35 * horizontal_distance,
        ) * alignment

        self.get_logger().info(
            f'WP{self.waypoint_index}: d={horizontal_distance:.1f} m, '
            f'prof={-self.z:.1f}/{target_depth:.1f} m, '
            f'errore prua={math.degrees(yaw_error):.0f} deg',
            throttle_duration_sec=1.0,
        )
        return desired_u, 0.0, desired_yaw_rate, target_depth

    def depth_control(self, target_depth):
        target_depth = clamp(target_depth, 0.5, self.max_depth)
        target_z = -target_depth
        error_z = target_z - self.z

        # Outer position loop produces a physically plausible vertical speed;
        # inner velocity loop plus a bounded integral rejects slight buoyancy.
        desired_w = clamp(
            0.45 * error_z,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )
        self.depth_integral = clamp(
            self.depth_integral + error_z * self.dt, -2.0, 2.0)
        return clamp(
            1.5 * (desired_w - self.w) + 0.10 * self.depth_integral,
            -1.0,
            1.0,
        )

    @staticmethod
    def allocate_thrusters(surge, sway, heave, yaw):
        """Allocate body commands to the BlueROV2 Heavy's eight thrusters."""
        commands = [
            surge + sway + yaw,
            surge - sway - yaw,
            surge - sway + yaw,
            surge + sway - yaw,
            heave,
            -heave,
            -heave,
            heave,
        ]
        peak = max(1.0, max(abs(value) for value in commands))
        return [value / peak for value in commands]

    def publish_motors(self, desired):
        max_step = self.motor_slew_rate * self.dt
        for index, target in enumerate(desired):
            delta = clamp(
                target - self.motor_command[index], -max_step, max_step)
            self.motor_command[index] = clamp(
                self.motor_command[index] + delta, -1.0, 1.0)

        # The SDF ESC range is 1100..1900 us with 1500 us neutral.
        message = Actuators()
        message.header.stamp = self.get_clock().now().to_msg()
        message.velocity = [
            1500.0 + 400.0 * value for value in self.motor_command
        ]
        self.actuator_pub.publish(message)

    def control_loop(self):
        self.handle_keyboard()
        if not self.odom_received:
            self.get_logger().info(
                'In attesa dell odometria UUV...', throttle_duration_sec=2.0)
            self.publish_motors([0.0] * 8)
            return

        if self.mode == 'waypoint':
            target_u, target_v, target_r, target_depth = (
                self.waypoint_targets())
            if target_depth is None:
                target_depth = -self.z
            self.target_depth = target_depth
        else:
            target_u = self.manual_u
            target_v = self.manual_v
            target_r = self.manual_yaw_rate
            target_depth = self.target_depth

        # Inner velocity loops. Hydrodynamic damping remains simulated by
        # Gazebo; these gains only determine the demanded normalized thrust.
        surge = clamp(1.1 * (target_u - self.u), -1.0, 1.0)
        sway = clamp(1.3 * (target_v - self.v), -1.0, 1.0)
        yaw = clamp(1.0 * (target_r - self.yaw_rate), -1.0, 1.0)
        heave = self.depth_control(target_depth)
        self.publish_motors(
            self.allocate_thrusters(surge, sway, heave, yaw))

    def stop(self):
        """Send neutral repeatedly so the bridge delivers the safe command."""
        if not rclpy.ok():
            return
        self.motor_command = [0.0] * 8
        for _ in range(3):
            self.publish_motors([0.0] * 8)

    def restore_terminal(self):
        if self.terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN, self.terminal_settings)
            self.terminal_settings = None


def main(args=None):
    rclpy.init(args=args)
    node = UuvController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
