#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class SonarVisualizer(Node):
    def __init__(self):
        super().__init__('sonar_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/vessel_v2/sonar/points',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Convertiamo il messaggio PointCloud2 in una lista di punti (X, Y, Z)
        punti = list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))
        
        if punti:
            # Prendiamo il raggio centrale del cono (il punto che punta dritto verso il basso)
            indice_centrale = len(punti) // 2
            x, y, z = punti[indice_centrale]
            
            # Nei sensori di Gazebo, l'asse Z esprime la distanza dal sensore.
            # Essendo rivolto verso il basso, il valore estratto è negativo, lo rendiamo assoluto.
            distanza_metri = abs(z)
            distanza = math.sqrt(x*x + y*y + z*z)
            # self.get_logger().info(f'Fondale intercettato! Distanza reale sotto lo scafo: {distanza_metri:.2f} metri')
            # self.get_logger().info(    f"x={x:.3f} y={y:.3f} z={z:.3f} dist={distanza:.3f}")
            self.get_logger().info(f"Numero punti: {len(punti)}")

            min_dist = 1e9
            best = None

            for p in punti:
                x,y,z = p
                d = math.sqrt(x*x+y*y+z*z)

                if d < min_dist:
                    min_dist = d
                    best = (x,y,z)

            self.get_logger().info(
                f"MIN -> x={best[0]:.3f} y={best[1]:.3f} z={best[2]:.3f} d={min_dist:.3f}"
                )
            
            finite_points = []

            for p in punti:
                x,y,z = p

                if (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    finite_points.append((x,y,z))

            self.get_logger().info(
                f"Punti validi: {len(finite_points)}"
            )

            min_x = min(p[0] for p in finite_points)
            max_x = max(p[0] for p in finite_points)

            min_y = min(p[1] for p in finite_points)
            max_y = max(p[1] for p in finite_points)

            min_z = min(p[2] for p in finite_points)
            max_z = max(p[2] for p in finite_points)

            self.get_logger().info(
                f"X:[{min_x:.2f},{max_x:.2f}] "
                f"Y:[{min_y:.2f},{max_y:.2f}] "
                f"Z:[{min_z:.2f},{max_z:.2f}]"
            )

            for i in range(5):
                x,y,z = finite_points[i]
                self.get_logger().info(
                    f"P{i}: x={x:.3f} y={y:.3f} z={z:.3f}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = SonarVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()