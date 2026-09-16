import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import math

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        
        # Origine della simulazione (da SDF)
        self.lat_ref = 36.509513
        self.lon_ref = -6.263074
        
        # Inserisci qui i tuoi Waypoint in GPS
        self.waypoints_gps = [
            (36.509800, -6.262500), 
            (36.510500, -6.261000), 
            (36.509250, -6.260360)  
        ]
        
        # Converti i target in Metri all'avvio
        self.waypoints_enu = [self.latlon_to_enu(lat, lon) for lat, lon in self.waypoints_gps]
        
        self.current_wp_index = 0
        self.r_acceptance = 15.0 # Metri
        
        # Posizione attuale
        self.current_x = 0.0
        self.current_y = 0.0
        self.gps_received = False
        
        # TOPIC AGGIORNATI
        self.gps_sub = self.create_subscription(NavSatFix, '/vessel_v2/gps/fix', self.gps_callback, 10)
        self.target_pub = self.create_publisher(Point, '/target_waypoint', 10)
        
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
        # Converti la posizione GPS attuale nei metri locali (così non abbiamo drift!)
        self.current_x, self.current_y = self.latlon_to_enu(msg.latitude, msg.longitude)
        self.gps_received = True

    def check_waypoint_loop(self):
        if not self.gps_received or self.current_wp_index >= len(self.waypoints_enu):
            return
            
        wp_x, wp_y = self.waypoints_enu[self.current_wp_index]
        
        distanza = math.sqrt((wp_x - self.current_x)**2 + (wp_y - self.current_y)**2)
        
        if distanza <= self.r_acceptance:
            self.get_logger().info(f'Waypoint {self.current_wp_index} raggiunto!')
            self.current_wp_index += 1
            if self.current_wp_index < len(self.waypoints_enu):
                wp_x, wp_y = self.waypoints_enu[self.current_wp_index]
        
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