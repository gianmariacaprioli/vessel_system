#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix


class X500WaypointManager(Node):

    def __init__(self):
        super().__init__('x500_waypoint_manager')

        # ------------------------------------------------------------
        # GPS origin of the Gazebo world
        # ------------------------------------------------------------
        self.lat0 = 36.509513
        self.lon0 = -6.263074

        # ------------------------------------------------------------
        # GPS waypoints: lat, lon
        # ------------------------------------------------------------
        self.waypoints_gps = [
            (36.509800, -6.262500),
            (36.510500, -6.261000),
            (36.509250, -6.260360),
        ]

        default_waypoints = [
            self.gps_to_enu(lat, lon)
            for lat, lon in self.waypoints_gps
        ]
        coordinates = self.declare_parameter(
            'waypoints_xy', [value for point in default_waypoints for value in point]
        ).value
        if not coordinates or len(coordinates) % 2 or not all(map(math.isfinite, coordinates)):
            raise ValueError('waypoints_xy must contain finite ENU x/y pairs')
        self.waypoints = list(zip(coordinates[::2], coordinates[1::2]))

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.position = None
        self.yaw = 0.0

        self.current_wp = 0

        self.takeoff_altitude = self.declare_parameter('takeoff_altitude', 10.0).value

        self.takeoff_delay = self.declare_parameter('takeoff_delay', 0.0).value
        self.takeoff_ready_at = None
        self.takeoff_bow_radius = self.declare_parameter('takeoff_bow_radius', 0.0).value
        self.vessel_bow_offset = self.declare_parameter('vessel_bow_offset', 0.0).value
        self.takeoff_vessel_radius = self.declare_parameter('takeoff_vessel_radius', 0.0).value
        self.column_orbit_radius = self.declare_parameter('column_orbit_radius', 0.0).value
        self.takeoff_xy = None
        self.takeoff_released = max(self.takeoff_bow_radius, self.takeoff_vessel_radius) <= 0.0
        if not self.takeoff_released:
            self.create_subscription(
                Odometry, '/model/vessel_v2/odometry', self.vessel_odom_callback, 10,
            )
        self.takeoff_done = False
        self.finished = False

        # Tolerances
        self.position_tolerance = self.declare_parameter('position_tolerance', 3.0).value
        self.altitude_tolerance = 0.5       # meters

        # Velocity limits
        self.max_horizontal_velocity = self.declare_parameter(
            'max_horizontal_velocity', 3.0).value
        self.max_vertical_velocity = 1.0    # m/s

        # Proportional gain
        self.kp_xy = 0.5
        self.kp_z = 0.5

        # ------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/x500/cmd_vel',
            10
        )

        self.enable_pub = self.create_publisher(
            Bool,
            '/x500/enable',
            10
        )

        if self.declare_parameter('use_odometry', False).value:
            self.position_sub = self.create_subscription(
                Odometry, '/model/x500/odometry', self.odom_callback, 10)
        else:
            self.position_sub = self.create_subscription(
                NavSatFix, '/x500/gps/fix', self.gps_callback, 10)

        # 10 Hz control loop
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('X500 waypoint manager started')

        if self.column_orbit_radius <= 0.0:
            for i, (x, y) in enumerate(self.waypoints):
                self.get_logger().info(f'WP{i}: E={x:.1f} m N={y:.1f} m')

    # ------------------------------------------------------------
    # GPS -> local ENU
    # ------------------------------------------------------------
    def gps_to_enu(self, lat, lon):

        meters_per_deg_lat = 110540.0

        meters_per_deg_lon = (
            111320.0 *
            math.cos(math.radians(self.lat0))
        )

        east = (
            (lon - self.lon0) *
            meters_per_deg_lon
        )

        north = (
            (lat - self.lat0) *
            meters_per_deg_lat
        )

        return east, north

    # ------------------------------------------------------------
    # GPS callback
    # ------------------------------------------------------------
    def gps_callback(self, msg):

        if all(map(math.isfinite, (msg.latitude, msg.longitude, msg.altitude))):
            self.position = (*self.gps_to_enu(msg.latitude, msg.longitude), msg.altitude)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        if all(map(math.isfinite, (position.x, position.y, position.z))):
            self.position = (position.x, position.y, position.z)
            self.yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def vessel_odom_callback(self, msg):
        if self.takeoff_released:
            return
        pose = msg.pose.pose
        if self.takeoff_vessel_radius > 0.0:
            radius = math.hypot(pose.position.x, pose.position.y)
            if radius <= self.takeoff_vessel_radius:
                self.takeoff_released = True
                self.get_logger().info(f'Takeoff released: vessel at {radius:.2f} m from the tower axis')
            return
        yaw = self.yaw_from_quaternion(pose.orientation)
        bow_x = pose.position.x + self.vessel_bow_offset * math.cos(yaw)
        bow_y = pose.position.y + self.vessel_bow_offset * math.sin(yaw)
        radius = math.hypot(bow_x, bow_y)
        if radius <= self.takeoff_bow_radius:
            self.takeoff_released = True
            self.get_logger().info(f'Takeoff released: vessel bow {radius:.2f} m from world origin')

    # ------------------------------------------------------------
    # Publish enable
    # ------------------------------------------------------------
    def enable_drone(self):

        msg = Bool()
        msg.data = True

        self.enable_pub.publish(msg)

    # ------------------------------------------------------------
    # Publish velocity
    # ------------------------------------------------------------
    def publish_velocity(self, vx, vy, vz):

        msg = Twist()

        # Gazebo accepts horizontal velocity in the vehicle heading frame.
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        msg.linear.x = cos_yaw * vx + sin_yaw * vy
        msg.linear.y = -sin_yaw * vx + cos_yaw * vy
        msg.linear.z = vz

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------
    # Clamp
    # ------------------------------------------------------------
    def clamp(self, value, minimum, maximum):

        return max(minimum, min(maximum, value))

    # ------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------
    def control_loop(self):

        if self.position is None:
            self.get_logger().info(
                'Waiting for position...',
                throttle_duration_sec=2.0
            )
            return

        if self.takeoff_ready_at is None:
            self.takeoff_ready_at = self.get_clock().now().nanoseconds + int(
                self.takeoff_delay * 1e9
            )
        if self.get_clock().now().nanoseconds < self.takeoff_ready_at:
            return

        if not self.takeoff_released:
            return

        self.enable_drone()
        x, y, z = self.position

        # --------------------------------------------------------
        # TAKEOFF
        # --------------------------------------------------------
        if not self.takeoff_done:
            if self.column_orbit_radius > 0.0 and self.takeoff_xy is None:
                self.takeoff_xy = (x, y)
                angle = math.atan2(y, x)
                self.waypoints = [
                    (
                        self.column_orbit_radius * math.cos(angle + math.tau * step / 16),
                        self.column_orbit_radius * math.sin(angle + math.tau * step / 16),
                    )
                    for step in range(17)
                ] + [self.takeoff_xy]
                self.get_logger().info(f'Column orbit planned from takeoff point ({x:.2f}, {y:.2f})')

            error_z = self.takeoff_altitude - z

            if abs(error_z) < self.altitude_tolerance:

                self.takeoff_done = True

                self.publish_velocity(0.0, 0.0, 0.0)

                self.get_logger().info(
                    f'Takeoff complete: altitude={z:.2f} m'
                )

                return

            vz = self.kp_z * error_z

            vz = self.clamp(
                vz,
                -self.max_vertical_velocity,
                self.max_vertical_velocity
            )

            self.publish_velocity(
                0.0,
                0.0,
                vz
            )

            self.get_logger().info(
                f'Takeoff: altitude={z:.2f} '
                f'target={self.takeoff_altitude:.2f}',
                throttle_duration_sec=1.0
            )

            return

        # --------------------------------------------------------
        # ALL WAYPOINTS COMPLETED
        # --------------------------------------------------------
        if self.current_wp >= len(self.waypoints):
            if self.takeoff_xy is not None:
                home_x, home_y = self.takeoff_xy
                self.publish_velocity(
                    self.clamp(self.kp_xy * (home_x - x), -1.0, 1.0),
                    self.clamp(self.kp_xy * (home_y - y), -1.0, 1.0),
                    self.clamp(self.kp_z * (self.takeoff_altitude - z), -1.0, 1.0),
                )
                if not self.finished:
                    self.finished = True
                    self.get_logger().info('Column orbit complete. Hovering above the takeoff point.')
                return

            if not self.finished:

                self.finished = True

                self.publish_velocity(
                    0.0,
                    0.0,
                    0.0
                )

                self.get_logger().info(
                    'All waypoints completed.'
                )

            return

        # --------------------------------------------------------
        # CURRENT WAYPOINT
        # --------------------------------------------------------
        target_x, target_y = self.waypoints[self.current_wp]

        error_x = target_x - x
        error_y = target_y - y

        distance = math.sqrt(
            error_x ** 2 +
            error_y ** 2
        )

        # --------------------------------------------------------
        # Waypoint reached
        # --------------------------------------------------------
        if distance < self.position_tolerance:
            if self.takeoff_xy is None:
                self.publish_velocity(0.0, 0.0, 0.0)

            self.get_logger().info(
                f'WP{self.current_wp} reached '
                f'(distance={distance:.2f} m)'
            )

            self.current_wp += 1

            return

        # --------------------------------------------------------
        # Proportional velocity controller
        # --------------------------------------------------------
        vx = self.kp_xy * error_x
        vy = self.kp_xy * error_y

        # Limit horizontal speed
        speed = math.sqrt(
            vx ** 2 +
            vy ** 2
        )

        if speed > self.max_horizontal_velocity:

            scale = (
                self.max_horizontal_velocity /
                speed
            )

            vx *= scale
            vy *= scale

        # Maintain altitude
        vz = self.kp_z * (
            self.takeoff_altitude - z
        )

        vz = self.clamp(
            vz,
            -self.max_vertical_velocity,
            self.max_vertical_velocity
        )

        self.publish_velocity(
            vx,
            vy,
            vz
        )

        self.get_logger().info(
            f'WP{self.current_wp}: '
            f'pos=({x:.1f}, {y:.1f}, {z:.1f}) '
            f'error=({error_x:.1f}, {error_y:.1f}) '
            f'd={distance:.1f} m',
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = X500WaypointManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_velocity(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
