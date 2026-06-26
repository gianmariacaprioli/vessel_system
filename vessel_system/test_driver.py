import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# topic_rx = '/model/vessel_v1/joint/right_engine_joint/cmd_thrust'
# topic_lx = '/model/vessel_v1/joint/left_engine_joint/cmd_thrust'

class TestDriver(Node):
    def __init__(self):
        super().__init__('test_driver')
        
        # Publisher per i motori del WAM-V
        self.left_pub = self.create_publisher(Float64, '/model/vessel_v2/joint/right_engine_joint/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/vessel_v2/joint/left_engine_joint/cmd_thrust', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = 20000.0
        
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)
        
        self.get_logger().info(f'Test Driver: Inviata spinta di {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TestDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
