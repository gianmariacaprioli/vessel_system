import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Wrench
import math

from .surface_models import declare_model

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.lat_ref = 36.509513
        self.lon_ref = -6.263074

        model, profile = declare_model(self)
        for name in ('mass', 'xU', 'xUabsU', 'nR', 'nRabsR', 'kp_surge',
                     'kd_surge', 'kp_yaw', 'kd_yaw', 'max_speed', 'use_gps'):
            setattr(self, name, self.declare_parameter(name, profile[name]).value)
        
        # --- MODELLO MASSA-MOLLA-SMORZATORE ---
        self.wp_initial_distance = 0.0 # Il nostro "target" (Distanza totale da percorrere)
        self.x_d = 0.0  # Metri percorsi dalla traiettoria ideale
        self.v_d = 0.0  # Velocità della traiettoria ideale
        
        # Frequenza naturale (omega_n) e smorzamento (zeta)
        self.omega_n = self.declare_parameter('omega_n', profile.get('omega_n', 0.1)).value
        self.yaw_drag_feedforward = self.declare_parameter(
            'yaw_drag_feedforward', profile.get('yaw_drag_feedforward', 1.0)).value
        self.approach_gain = self.declare_parameter(
            'approach_gain', profile.get('approach_gain', 0.0)).value
        self.turn_braking_gain = self.declare_parameter('turn_braking_gain', 0.0).value
        self.turn_heading_tolerance = self.declare_parameter('turn_heading_tolerance', 0.45).value
        self.waypoint_tolerance = self.declare_parameter(
            'waypoint_tolerance', profile.get('waypoint_tolerance', 0.0)).value
        self.zeta = 1.0    # Smorzamento critico (nessuna oscillazione)
        
        # Stato nave
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_u = 0.0 # Velocità X
        self.current_r = 0.0 # Velocità rotazione Z
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_received = False
        self.waypoint_reached = False
        self.position_received = False
        self.odom_received = False
        self.dt = 0.1
        
        # Sottoscrizioni
        if self.use_gps:
            gps_topic = self.declare_parameter('gps_topic', f'/{model}/gps/fix').value
            self.gps_sub = self.create_subscription(NavSatFix, gps_topic, self.gps_callback, 10)
        odom_topic = self.declare_parameter('odom_topic', f'/model/{model}/odometry').value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.target_sub = self.create_subscription(Point, 'target_waypoint', self.target_callback, 10)
        self.wrench_pub = self.create_publisher(Wrench, 'commanded_wrench', 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)

    def latlon_to_enu(self, lat, lon):
        R_earth = 6378137.0 
        lat_rad, lon_rad = math.radians(lat), math.radians(lon)
        lat_ref_rad, lon_ref_rad = math.radians(self.lat_ref), math.radians(self.lon_ref)
        x = R_earth * (lon_rad - lon_ref_rad) * math.cos(lat_ref_rad)
        y = R_earth * (lat_rad - lat_ref_rad)                         
        return x, y

    def gps_callback(self, msg):
        if not all(map(math.isfinite, (msg.latitude, msg.longitude))):
            return
        self.current_x, self.current_y = self.latlon_to_enu(msg.latitude, msg.longitude)
        self.position_received = True

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        if not self.use_gps:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.position_received = True
        self.odom_received = True
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.current_u = msg.twist.twist.linear.x 
        self.current_r = msg.twist.twist.angular.z 

    def target_callback(self, msg):
        if not all(map(math.isfinite, (msg.x, msg.y))):
            return
        # Se riceviamo un NUOVO waypoint, resettiamo il modello di riferimento
        if not self.target_received or msg.x != self.target_x or msg.y != self.target_y:
            self.target_x = msg.x
            self.target_y = msg.y
            
            # 1. Il target del modello diventa la distanza totale fino al waypoint
            self.wp_initial_distance = None
            
            # 2. Resettiamo le condizioni iniziali della molla
            self.x_d = 0.0
            self.target_received = True
            self.waypoint_reached = False

    def control_loop(self):
        if not (self.target_received and self.position_received and self.odom_received):
            return

        distance = math.hypot(self.target_x - self.current_x,
                              self.target_y - self.current_y)
        if self.waypoint_tolerance > 0.0 and distance <= self.waypoint_tolerance:
            self.waypoint_reached = True
        if self.waypoint_reached:
            wrench = Wrench()
            wrench.force.x = -max(self.kd_surge, self.turn_braking_gain) * self.current_u
            wrench.torque.z = -self.kd_yaw * self.current_r
            self.wrench_pub.publish(wrench)
            return

        # ----------------------------------------------------
        # 1. ERRORE YAW (Rotazione verso il bersaglio)
        # ----------------------------------------------------
        desired_yaw = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Close to obstacles, finish the turn before accelerating into the next leg.
        if self.turn_braking_gain > 0.0 and abs(yaw_error) > self.turn_heading_tolerance:
            wrench = Wrench()
            wrench.force.x = -self.turn_braking_gain * self.current_u
            wrench.torque.z = self.kp_yaw * yaw_error - self.kd_yaw * self.current_r
            self.wrench_pub.publish(wrench)
            self.wp_initial_distance = None
            self.x_d = 0.0
            return

        if self.wp_initial_distance is None:
            self.wp_initial_distance = distance
            self.v_d = self.current_u

        # ----------------------------------------------------
        # 2. MODELLO DI RIFERIMENTO (Avanzamento X)
        # ----------------------------------------------------
        # Se la prua è fuori bersaglio di oltre ~25 gradi, sospendiamo 
        # l'avanzamento del punto virtuale per aspettare la virata
        if abs(yaw_error) > 0.45:
            a_d = -2.0 * self.v_d # Frena il punto virtuale
        else:
            # MASSA-MOLLA-SMORZATORE ESATTO!
            a_d = (self.omega_n**2) * (self.wp_initial_distance - self.x_d) - (2 * self.zeta * self.omega_n * self.v_d)
        
        # Saturazione accelerazione (evita picchi assurdi all'avvio)
        a_d = max(min(a_d, 0.5), -0.5) 
        if self.approach_gain > 0.0:
            approach_speed = min(self.max_speed, self.approach_gain * distance)
            speed_limit_accel = (approach_speed - self.v_d) / self.dt
            a_d = max(-0.5, min(a_d, speed_limit_accel))

        # Integrazione di Eulero (Equazioni cinematiche)
        self.v_d += a_d * self.dt
        self.v_d = max(min(self.v_d, self.max_speed), 0.0)
        self.x_d += self.v_d * self.dt

        # ----------------------------------------------------
        # 3. FEEDFORWARD (Sfruttiamo l'idrodinamica di Gazebo)
        # ----------------------------------------------------
        # Calcoliamo quanto frena l'acqua alla velocità IDEALE
        drag_x = (self.xU * self.v_d) + (self.xUabsU * abs(self.v_d) * self.v_d)
        
        # Tau FF (Avanzamento): Forza per accelerare + Forza per annullare il drag negativo
        tau_FF_surge = (self.mass * a_d) - drag_x 

        # Calcoliamo quanto frena l'acqua durante la rotazione REALE
        drag_yaw = (self.nR * self.current_r) + (self.nRabsR * abs(self.current_r) * self.current_r)
        
        # Tau FF (Rotazione): Annulliamo l'attrito dell'acqua
        tau_FF_yaw = -self.yaw_drag_feedforward * drag_yaw

        # ----------------------------------------------------
        # 4. FEEDBACK (PID per correggere disturbi esterni)
        # ----------------------------------------------------
        # Quanta distanza REALE abbiamo percorso?
        actual_distance_traveled = self.wp_initial_distance - distance
        
        # Errore rispetto al modello (Molla virtuale)
        error_x = self.x_d - actual_distance_traveled
        error_v = self.v_d - self.current_u
        
        tau_PID_surge = (self.kp_surge * error_x) + (self.kd_surge * error_v)
        tau_PID_yaw = (self.kp_yaw * yaw_error) + (self.kd_yaw * (0.0 - self.current_r))

        # ----------------------------------------------------
        # 5. PUBBLICAZIONE WRENCH
        # ----------------------------------------------------
        wrench_msg = Wrench()
        wrench_msg.force.x = tau_FF_surge + tau_PID_surge
        wrench_msg.torque.z = tau_FF_yaw + tau_PID_yaw
        self.wrench_pub.publish(wrench_msg)
        
        # Log di debug
        self.get_logger().debug(f"Dist. Reale: {actual_distance_traveled:.1f} | Dist. Ideale (x_d): {self.x_d:.1f} | v_d: {self.v_d:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
