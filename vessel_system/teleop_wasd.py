#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import select
import tty
import termios

msg = """
--------------------------------------------------
ASV Twin-Engine Teleop - Pilota Manuale
--------------------------------------------------
Controllo Diretto dei Motori (Tank Steer):

        W : Accelera entrambi
   A : Gira a SX   S : Frena/Indietro   D : Gira a DX

Spazio o X : Azzera le manette (Motori a 0)
CTRL+C     : Esci dal pilota automatico

Nota: La spinta non si azzera da sola. È come una vera manetta!
--------------------------------------------------
"""

# Configurazione della spinta
MAX_THRUST = 100000000.0    # Spinta massima applicabile a un singolo motore (Newton)
THRUST_STEP = 500.0     # Quanto aumenta la spinta lineare a ogni pressione di W/S
TURN_STEP =  1500.0     # Spinta di sterzata (Aumentata per vincere l'attrito dell'acqua)

def get_key(settings):
    """Cattura il singolo tasto premuto sul terminale"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

class TwinEngineTeleop(Node):
    def __init__(self):
        super().__init__('teleop_wasd_twin_engine')
        
        # Publisher separati per il motore sinistro e destro
        self.pub_left = self.create_publisher(Float64, '/model/vessel_v2/joint/left_engine_joint/cmd_thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/model/vessel_v2/joint/right_engine_joint/cmd_thrust', 10)
        
        # Manette reali dei due motori
        self.thrust_left = 0.0
        self.thrust_right = 0.0
        
        self.get_logger().info("Motori accesi! Pronto a navigare.")

    def update_thrust(self, key):
        # Accelerazione (Entrambi i motori avanti)
        if key == 'w' or key == 'W':
            self.thrust_left = min(self.thrust_left + THRUST_STEP, MAX_THRUST)
            self.thrust_right = min(self.thrust_right + THRUST_STEP, MAX_THRUST)
            
        # Frenata/Retromarcia (Entrambi i motori indietro)
        elif key == 's' or key == 'S':
            self.thrust_left = max(self.thrust_left - THRUST_STEP, -MAX_THRUST)
            self.thrust_right = max(self.thrust_right - THRUST_STEP, -MAX_THRUST)
            
        # Virata a Sinistra: Motore Destro spinge, Motore Sinistro frena
        elif key == 'a' or key == 'A':
            self.thrust_left = min(self.thrust_left - TURN_STEP, -MAX_THRUST)
            self.thrust_right = max(self.thrust_right + TURN_STEP, MAX_THRUST)
            
        # Virata a Destra: Motore Sinistro spinge, Motore Destro frena
        elif key == 'd' or key == 'D':
            self.thrust_left = max(self.thrust_left + TURN_STEP, MAX_THRUST)
            self.thrust_right = min(self.thrust_right - TURN_STEP, -MAX_THRUST)
            
        # Stop d'emergenza
        elif key == ' ' or key == 'x' or key == 'X':
            self.thrust_left = 0.0
            self.thrust_right = 0.0

        # Pubblicazione sui topic
        msg_l = Float64()
        msg_l.data = self.thrust_left
        msg_r = Float64()
        msg_r.data = self.thrust_right
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)
        
        # Stampa a schermo per farti capire esattamente cosa stanno facendo i motori
        print(f"Motore SX: {self.thrust_left:7.1f} N  |  Motore DX: {self.thrust_right:7.1f} N   ", end="\r")

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    
    node = TwinEngineTeleop()
    print(msg)
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key: # Aggiorna la spinta SOLO se viene premuto un tasto
                node.update_thrust(key)
            if key == '\x03': # Intercetta il CTRL+C per uscire
                break
                
    except Exception as e:
        print(f"Errore: {e}")
        
    finally:
        # Ferma i motori quando chiudi lo script
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.pub_left.publish(stop_msg)
        node.pub_right.publish(stop_msg)
        
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()