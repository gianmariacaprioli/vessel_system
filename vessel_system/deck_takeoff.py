#!/usr/bin/env python3
"""Wait for the X500 to settle on the deck and then climb to a hover."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class DeckTakeoff(Node):
    def __init__(self):
        super().__init__('deck_takeoff')
        self.vessel = None
        self.drone = None
        self.target_z = None
        self.settled_since = None
        self.hover_reported = False

        self.create_subscription(
            Odometry, '/model/vessel_v2/odometry', self.receive_vessel,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/model/x500/odometry', self.receive_drone,
            qos_profile_sensor_data,
        )
        self.enable = self.create_publisher(Bool, '/x500/enable', 10)
        self.velocity = self.create_publisher(Twist, '/x500/cmd_vel', 10)
        self.create_timer(0.1, self.update)

    def receive_vessel(self, message):
        self.vessel = message

    def receive_drone(self, message):
        self.drone = message

    def update(self):
        if self.drone is None or self.vessel is None:
            return
        drone_z = self.drone.pose.pose.position.z
        if self.target_z is None:
            if not (
                self.enable.get_subscription_count() and self.velocity.get_subscription_count()
            ):
                return
            vessel_pose = self.vessel.pose.pose.position
            drone_pose = self.drone.pose.pose.position
            relative_vertical_speed = (
                self.drone.twist.twist.linear.z - self.vessel.twist.twist.linear.z
            )
            # The hull collision box has its upper face 1.02 m above the origin.
            on_deck = (
                abs(drone_pose.x - vessel_pose.x + 2.5) < 1.0
                and abs(drone_pose.y - vessel_pose.y) < 1.0
                and 0.8 < drone_z - vessel_pose.z < 1.4
                and abs(relative_vertical_speed) < 0.15
            )
            stamp = self.drone.header.stamp
            now = stamp.sec + stamp.nanosec * 1e-9
            if not on_deck:
                self.settled_since = None
                return
            if self.settled_since is None:
                self.settled_since = now
                return
            if now - self.settled_since < 0.5:
                return
            self.target_z = drone_z + 5.0
            self.get_logger().info('X500 on deck. Taking off to 5 m above the deck.')

        error = self.target_z - drone_z
        command = Twist()
        command.linear.z = max(-1.0, min(1.0, 0.8 * error))
        self.enable.publish(Bool(data=True))
        self.velocity.publish(command)
        if not self.hover_reported and abs(error) < 0.15:
            self.get_logger().info('X500 reached hover altitude.')
            self.hover_reported = True


def main():
    rclpy.init()
    node = DeckTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
