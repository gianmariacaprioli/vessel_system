#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
from collections import deque

class GpsVelocityEstimator(Node):
    def __init__(self):
        super().__init__('gps_velocity_estimator')
        
        # Sottoscrizione al topic GPS
        self.subscription = self.create_subscription(
            NavSatFix,
            '/vessel_v2/gps/fix',
            self.gps_callback,
            10)
            
        # Variabili di stato per il calcolo
        self.last_lat = None
        self.last_lon = None
        self.last_time = None
        
        # Finestra per la media mobile (stabilizza la lettura della velocità)
        self.velocity_window = deque(maxlen=10)
        
        self.get_logger().info("Nodo stimatore velocità GPS avviato. In attesa di fix...")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calcola la grande distanza del cerchio tra due punti in metri usando Haversine"""
        # Raggio della Terra (WGS84 richiede un raggio medio molto preciso)
        R = 6371000.0 
        
        # Conversione in radianti
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        # Formula di Haversine
        a = math.sin(delta_phi / 2.0)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        
        return R * c

    def gps_callback(self, msg):
        # Ignora i dati se il GPS non ha un fix valido
        if msg.status.status == -1: # STATUS_NO_FIX
            return
            
        current_time = self.get_clock().now()
        current_lat = msg.latitude
        current_lon = msg.longitude
        
        # Se è il primissimo punto, salviamo i dati e aspettiamo il successivo
        if self.last_lat is None or self.last_lon is None or self.last_time is None:
            self.last_lat = current_lat
            self.last_lon = current_lon
            self.last_time = current_time
            return
            
        # Calcola il delta di tempo in secondi
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Evitiamo divisioni per zero se i messaggi arrivano troppo vicini o duplicati
        if dt < 0.01:
            return
            
        # Calcola la distanza in metri percorsa dall'ultimo fix
        distance = self.haversine_distance(self.last_lat, self.last_lon, current_lat, current_lon)
        
        # Calcola la velocità istantanea in m/s
        instantaneous_speed_ms = distance / dt
        
        # Filtro: Se Gazebo spawna o teletrasporta la nave, evitiamo picchi assurdi (es. > 50 m/s)
        if instantaneous_speed_ms < 50.0:
            self.velocity_window.append(instantaneous_speed_ms)
            
        # Calcola la media mobile se abbiamo abbastanza campioni
        if len(self.velocity_window) > 0:
            avg_speed_ms = sum(self.velocity_window) / len(self.velocity_window)
        else:
            avg_speed_ms = instantaneous_speed_ms
            
        # conversioni delle unità di misura
        speed_kmh = avg_speed_ms * 3.6
        speed_knots = avg_speed_ms * 1.94384
        
        # Stampa a schermo la dashboard delle velocità in tempo reale
        print(f"Velocità Nave ->  M/S: {avg_speed_ms:5.2f} m/s  |  KM/H: {speed_kmh:5.2f} km/h  |  NODI: {speed_knots:5.2f} kn    ", end="\r")
        
        # Aggiorna lo stato per il prossimo ciclo
        self.last_lat = current_lat
        self.last_lon = current_lon
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = GpsVelocityEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nArresto del nodo estimatore.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()