import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
import math

class CustomOdometry(Node):
    def __init__(self):
        super().__init__('custom_odometry')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.origin_lat = None
        self.origin_lon = None
        self.current_orientation = None
        
        self.imu_sub = self.create_subscription(
            Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, qos_profile_sensor_data)
            
        self.gps_sub = self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, qos_profile_sensor_data)
            
        self.get_logger().info('Odometria Custom avviata. In attesa dei sensori GPS e IMU...')

    def imu_callback(self, msg):
        self.current_orientation = msg.orientation

    def gps_callback(self, msg):
        if self.current_orientation is None:
            return

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info('Segnale GPS agganciato! Pubblicazione TF odom -> base_link in corso.')

        lat_rad = math.radians(self.origin_lat)
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 40075000.0 * math.cos(lat_rad) / 360.0

        dx = (msg.longitude - self.origin_lon) * meters_per_deg_lon
        dy = (msg.latitude - self.origin_lat) * meters_per_deg_lat

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'wamv/wamv/base_link'
        
        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = 0.0
        t.transform.rotation = self.current_orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CustomOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()