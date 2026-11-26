"""Follow one continuous ENU surface route using odometry and body-frame wrench."""

from bisect import bisect_right
import math

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from .surface_models import declare_model


class SurfacePathFollower(Node):
    def __init__(self):
        super().__init__('surface_path_follower')
        model, self.profile = declare_model(self)
        coordinates = self.declare_parameter('path_xy', Parameter.Type.DOUBLE_ARRAY).value
        if (not coordinates or len(coordinates) < 4 or len(coordinates) % 2
                or not all(map(math.isfinite, coordinates))):
            raise ValueError('path_xy must contain at least two finite ENU points')
        self.points = list(zip(coordinates[::2], coordinates[1::2]))
        self.stations = [0.0]
        for start, end in zip(self.points, self.points[1:]):
            length = math.dist(start, end)
            if length == 0.0:
                raise ValueError('Adjacent path points must be distinct')
            self.stations.append(self.stations[-1] + length)
        self.speed = self.declare_parameter('speed', 1.6).value
        self.entry_speed = self.declare_parameter('entry_speed', self.speed).value
        self.entry_length = self.declare_parameter('entry_length', 0.0).value
        self.lookahead = self.declare_parameter('lookahead', 4.0).value
        self.position = None
        self.progress = 0.0
        self.segment = 0
        self.finished = False
        self.wrench_pub = self.create_publisher(Wrench, 'commanded_wrench', 10)
        self.create_subscription(Odometry, f'/model/{model}/odometry', self.receive_pose, 10)
        self.create_timer(0.1, self.update)

    def receive_pose(self, message):
        pose = message.pose.pose
        q = pose.orientation
        self.position = (pose.position.x, pose.position.y)
        self.yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.surge = message.twist.twist.linear.x
        self.yaw_rate = message.twist.twist.angular.z

    def update_progress(self):
        x, y = self.position
        closest = math.inf
        # Keep the projection local so nearby parts of the closed orbit
        # cannot skip the vessel straight to the finish.
        for index in range(self.segment, min(self.segment + 12, len(self.points) - 1)):
            ax, ay = self.points[index]
            bx, by = self.points[index + 1]
            dx, dy = bx - ax, by - ay
            length = self.stations[index + 1] - self.stations[index]
            fraction = min(1.0, max(0.0, ((x-ax)*dx + (y-ay)*dy) / length**2))
            distance = math.hypot(x - ax - fraction*dx, y - ay - fraction*dy)
            if distance < closest:
                closest = distance
                station = self.stations[index] + fraction * length
        self.progress = max(self.progress, station)
        self.segment = min(
            bisect_right(self.stations, self.progress) - 1, len(self.points) - 2,
        )

    def point_at(self, station):
        index = min(bisect_right(self.stations, station) - 1, len(self.points) - 2)
        start = self.stations[index]
        fraction = (station - start) / (self.stations[index + 1] - start)
        return tuple(
            a + fraction * (b - a) for a, b in zip(self.points[index], self.points[index + 1])
        )

    def update(self):
        if self.position is None:
            return

        self.update_progress()
        remaining = self.stations[-1] - self.progress
        endpoint_distance = math.dist(self.position, self.points[-1])
        command = Wrench()
        if self.finished or (remaining < 1.5 and endpoint_distance < 1.5):
            command.force.x = -20000.0 * self.surge
            command.torque.z = -35000.0 * self.yaw_rate
            if not self.finished:
                self.finished = True
                self.get_logger().info('Vessel completed one platform circuit.')
        else:
            x, y = self.position
            tx, ty = self.point_at(min(self.progress + self.lookahead, self.stations[-1]))
            bearing = math.atan2(ty-y, tx-x)
            angle = bearing - self.yaw
            heading_error = math.atan2(math.sin(angle), math.cos(angle))
            desired_yaw_rate = max(-0.25, min(0.25, 0.65 * heading_error))
            speed_limit = min(
                self.speed, self.entry_speed + 0.25 * max(0.0, self.progress - self.entry_length),
            )
            target_speed = min(speed_limit, math.sqrt(0.6 * max(remaining, endpoint_distance)))
            target_speed *= max(0.0, math.cos(heading_error))**2
            command.force.x = (
                self.profile['mass'] * 0.55 * (target_speed - self.surge)
                - self.profile['xU'] * target_speed
                - self.profile['xUabsU'] * target_speed**2
            )
            command.torque.z = (
                95000.0 * (desired_yaw_rate - self.yaw_rate)
                - self.profile['nR'] * desired_yaw_rate
                - self.profile['nRabsR'] * abs(desired_yaw_rate) * desired_yaw_rate
            )
        self.wrench_pub.publish(command)


def main():
    rclpy.init()
    node = SurfacePathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
