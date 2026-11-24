import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math

from .surface_models import declare_model

# Centres of waypoint_1, waypoint_2 and waypoint_3 in ocean_world.sdf (ENU metres).
OCEAN_WAYPOINTS = (51.36, 31.85, 185.57, 109.52, 242.81, -29.18)

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        model, profile = declare_model(self)
        self.use_gps = self.declare_parameter('use_gps', profile['use_gps']).value
        
        # Origine della simulazione (da SDF)
        self.lat_ref = 36.509513
        self.lon_ref = -6.263074
        
        coordinates = self.declare_parameter('waypoints_xy', list(OCEAN_WAYPOINTS)).value
        if not coordinates or len(coordinates) % 2 or not all(map(math.isfinite, coordinates)):
            raise ValueError('waypoints_xy must contain finite x/y pairs')
        self.waypoints = list(zip(coordinates[::2], coordinates[1::2]))
        
        self.current_wp_index = 0
        self.r_acceptance = self.declare_parameter(
            'acceptance_radius', 15.0 if self.use_gps else 0.35).value
        self.max_arrival_speed = self.declare_parameter('max_arrival_speed', 0.08).value
        self.speed = 0.0
        
        # Posizione attuale
        self.current_x = 0.0
        self.current_y = 0.0
        self.position_received = False
        
        # TOPIC AGGIORNATI
        if self.use_gps:
            topic = self.declare_parameter('gps_topic', f'/{model}/gps/fix').value
            self.position_sub = self.create_subscription(NavSatFix, topic, self.gps_callback, 10)
        else:
            topic = self.declare_parameter('odom_topic', f'/model/{model}/odometry').value
            self.position_sub = self.create_subscription(Odometry, topic, self.odom_callback, 10)
        self.target_pub = self.create_publisher(Point, 'target_waypoint', 10)
        
        self.timer = self.create_timer(0.1, self.check_waypoint_loop)

    def latlon_to_enu(self, lat, lon):
        R_earth = 6378137.0 
        lat_rad, lon_rad = math.radians(lat), math.radians(lon)
        lat_ref_rad, lon_ref_rad = math.radians(self.lat_ref), math.radians(self.lon_ref)
        d_lon = lon_rad - lon_ref_rad
        d_lat = lat_rad - lat_ref_rad
        x = R_earth * d_lon * math.cos(lat_ref_rad)
        y = R_earth * d_lat                         
        return x, y

    def gps_callback(self, msg):
        if not all(map(math.isfinite, (msg.latitude, msg.longitude))):
            return
        # Converti la posizione GPS attuale nei metri locali (così non abbiamo drift!)
        self.current_x, self.current_y = self.latlon_to_enu(msg.latitude, msg.longitude)
        self.position_received = True

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        self.current_x, self.current_y = position.x, position.y
        self.speed = math.hypot(velocity.x, velocity.y)
        self.position_received = True

    def check_waypoint_loop(self):
        if not self.position_received or self.current_wp_index >= len(self.waypoints):
            return
            
        wp_x, wp_y = self.waypoints[self.current_wp_index]
        
        distanza = math.sqrt((wp_x - self.current_x)**2 + (wp_y - self.current_y)**2)
        
        if distanza <= self.r_acceptance and (self.use_gps or self.speed <= self.max_arrival_speed):
            self.get_logger().info(f'Waypoint {self.current_wp_index} raggiunto!')
            self.current_wp_index += 1
            if self.current_wp_index == len(self.waypoints):
                self.get_logger().info('Missione waypoint completata.')
                return
            wp_x, wp_y = self.waypoints[self.current_wp_index]
        
        target_msg = Point()
        target_msg.x = float(wp_x)
        target_msg.y = float(wp_y)
        self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
