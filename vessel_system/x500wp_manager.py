#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
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

        # Convert GPS waypoints to local ENU
        self.waypoints = [
            self.gps_to_enu(lat, lon)
            for lat, lon in self.waypoints_gps
        ]

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        self.current_wp = 0

        self.takeoff_altitude = 10.0

        self.takeoff_done = False
        self.finished = False

        # Tolerances
        self.position_tolerance = 3.0       # meters
        self.altitude_tolerance = 0.5       # meters

        # Velocity limits
        self.max_horizontal_velocity = 3.0  # m/s
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

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/x500/gps/fix',
            self.gps_callback,
            10
        )

        # 10 Hz control loop
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('X500 waypoint manager started')

        for i, (x, y) in enumerate(self.waypoints):
            self.get_logger().info(
                f'WP{i}: E={x:.1f} m N={y:.1f} m'
            )

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

        if math.isnan(msg.latitude):
            return

        if math.isnan(msg.longitude):
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

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

        msg.linear.x = vx
        msg.linear.y = vy
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

        # Always keep the multicopter controller enabled
        self.enable_drone()

        # Wait for GPS
        if self.current_lat is None:
            self.get_logger().info(
                'Waiting for GPS...',
                throttle_duration_sec=2.0
            )
            return

        # Current local ENU position
        x, y = self.gps_to_enu(
            self.current_lat,
            self.current_lon
        )

        z = self.current_alt

        # --------------------------------------------------------
        # TAKEOFF
        # --------------------------------------------------------
        if not self.takeoff_done:

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

            self.publish_velocity(
                0.0,
                0.0,
                0.0
            )

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