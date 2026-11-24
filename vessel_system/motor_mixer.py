"""Allocate a body-frame wrench to the port and starboard actuators."""

import math

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Wrench
import rclpy
from rclpy.node import Node

from .surface_models import declare_model


class ControlAllocatorNode(Node):
    def __init__(self):
        super().__init__('control_allocator_node')
        model, profile = declare_model(self)
        for name in ('lever_arm', 'forward_thrust', 'reverse_thrust'):
            value = self.declare_parameter(name, profile[name]).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)

        actuator_topic = self.declare_parameter(
            'actuator_topic', f'/{model}/command/motor_speed').value
        self.motor_pub = self.create_publisher(Actuators, actuator_topic, 10)
        self.wrench_sub = self.create_subscription(
            Wrench, 'commanded_wrench', self.wrench_callback, 10)

    def thrust_to_command(self, thrust):
        full_scale = self.forward_thrust if thrust >= 0 else self.reverse_thrust
        return max(-1.0, min(1.0, thrust / full_scale))

    def wrench_callback(self, msg):
        surge = msg.force.x
        yaw = msg.torque.z
        command = Actuators()
        command.header.stamp = self.get_clock().now().to_msg()
        if math.isfinite(surge) and math.isfinite(yaw):
            differential = yaw / (2.0 * self.lever_arm)
            port = surge / 2.0 - differential
            starboard = surge / 2.0 + differential
            command.velocity = [
                self.thrust_to_command(port),
                self.thrust_to_command(starboard),
            ]
        else:
            command.velocity = [0.0, 0.0]
        self.motor_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
