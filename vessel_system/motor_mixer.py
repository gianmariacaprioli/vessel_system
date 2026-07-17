import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64

class ControlAllocatorNode(Node):
    def __init__(self):
        super().__init__('control_allocator_node')
        
        # Geometria della nave
        self.L_y = 2.0 # Distanza laterale dei motori dal centro in metri
        
        # Limiti Motore (Newton)
        self.max_thrust = 30000.0
        self.min_thrust = -20000.0
        
        # Pub / Sub
        self.wrench_sub = self.create_subscription(Wrench, '/commanded_wrench', self.wrench_callback, 10)
        self.right_thruster_pub = self.create_publisher(Float64, '/model/vessel_v2/joint/left_engine_joint/cmd_thrust', 10)
        self.left_thruster_pub = self.create_publisher(Float64, '/model/vessel_v2/joint/right_engine_joint/cmd_thrust', 10)

    def clip_thrust(self, thrust):
        return max(min(thrust, self.max_thrust), self.min_thrust)

    def wrench_callback(self, msg):
        forza_x_totale = msg.force.x
        momento_n_totale = msg.torque.z
        
        # Inversione matrice (Control Allocation)
        spinta_destro = (forza_x_totale / 2.0) + (momento_n_totale / (2.0 * self.L_y))
        spinta_sinistro = (forza_x_totale / 2.0) - (momento_n_totale / (2.0 * self.L_y))
        
        # Saturazione
        spinta_destro = self.clip_thrust(spinta_destro)
        spinta_sinistro = self.clip_thrust(spinta_sinistro)
        
        # Pubblicazione a Gazebo
        msg_right = Float64()
        msg_right.data = float(spinta_destro)
        self.right_thruster_pub.publish(msg_right)
        
        msg_left = Float64()
        msg_left.data = float(spinta_sinistro)
        self.left_thruster_pub.publish(msg_left)

def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()