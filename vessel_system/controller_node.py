import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Wrench
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.lat_ref = 36.509513
        self.lon_ref = -6.263074

        # --- PARAMETRI FISICI (Dal tuo file SDF) ---
        self.mass = 55000.0 # kg
        # Surge Drag (Resistenza in X)
        self.xU = -2750.0
        self.xUabsU = -1500.0
        # Yaw Drag (Resistenza in Rotazione Z)
        self.nR = -60922.0
        self.nRabsR = -283000.0
        
        # --- MODELLO MASSA-MOLLA-SMORZATORE ---
        self.wp_initial_distance = 0.0 # Il nostro "target" (Distanza totale da percorrere)
        self.x_d = 0.0  # Metri percorsi dalla traiettoria ideale
        self.v_d = 0.0  # Velocità della traiettoria ideale
        
        # Frequenza naturale (omega_n) e smorzamento (zeta)
        self.omega_n = 0.1 # Più è basso, più l'accelerazione è dolce
        self.zeta = 1.0    # Smorzamento critico (nessuna oscillazione)
        
        # --- GUADAGNI PID ---
        # Avendo un Feedforward perfetto, i Kp possono essere più bassi e stabili
        self.kp_surge = 2000.0
        self.kd_surge = 5000.0
        
        self.kp_yaw = 40000.0 # Lo yaw serve alto per vincere l'inerzia della rotazione
        self.kd_yaw = 20000.0
        
        # Stato nave
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_u = 0.0 # Velocità X
        self.current_r = 0.0 # Velocità rotazione Z
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_received = False
        self.gps_received = False
        self.dt = 0.1
        
        # Sottoscrizioni
        self.gps_sub = self.create_subscription(NavSatFix, '/vessel_v2/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/vessel_v2/odometry', self.odom_callback, 10)
        self.target_sub = self.create_subscription(Point, '/target_waypoint', self.target_callback, 10)
        self.wrench_pub = self.create_publisher(Wrench, '/commanded_wrench', 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)

    def latlon_to_enu(self, lat, lon):
        R_earth = 6378137.0 
        lat_rad, lon_rad = math.radians(lat), math.radians(lon)
        lat_ref_rad, lon_ref_rad = math.radians(self.lat_ref), math.radians(self.lon_ref)
        x = R_earth * (lon_rad - lon_ref_rad) * math.cos(lat_ref_rad)
        y = R_earth * (lat_rad - lat_ref_rad)                         
        return x, y

    def gps_callback(self, msg):
        self.current_x, self.current_y = self.latlon_to_enu(msg.latitude, msg.longitude)
        self.gps_received = True

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.current_u = msg.twist.twist.linear.x 
        self.current_r = msg.twist.twist.angular.z 

    def target_callback(self, msg):
        # Se riceviamo un NUOVO waypoint, resettiamo il modello di riferimento
        if msg.x != self.target_x or msg.y != self.target_y:
            self.target_x = msg.x
            self.target_y = msg.y
            
            # 1. Il target del modello diventa la distanza totale fino al waypoint
            self.wp_initial_distance = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)
            
            # 2. Resettiamo le condizioni iniziali della molla
            self.x_d = 0.0
            self.v_d = self.current_u
            self.target_received = True

    def control_loop(self):
        if not self.target_received or not self.gps_received:
            return

        # ----------------------------------------------------
        # 1. ERRORE YAW (Rotazione verso il bersaglio)
        # ----------------------------------------------------
        desired_yaw = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

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

        # Integrazione di Eulero (Equazioni cinematiche)
        self.v_d += a_d * self.dt
        self.v_d = max(min(self.v_d, 5.0), 0.0) # Saturo a max 5 m/s, vietata retromarcia
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
        tau_FF_yaw = -drag_yaw 

        # ----------------------------------------------------
        # 4. FEEDBACK (PID per correggere disturbi esterni)
        # ----------------------------------------------------
        # Quanta distanza REALE abbiamo percorso?
        current_dist_to_wp = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)
        actual_distance_traveled = self.wp_initial_distance - current_dist_to_wp
        
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
        self.get_logger().info(f"Dist. Reale: {actual_distance_traveled:.1f} | Dist. Ideale (x_d): {self.x_d:.1f} | v_d: {self.v_d:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()