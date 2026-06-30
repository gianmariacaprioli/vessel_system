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
Controllo Proporzionale dei Motori:

        W : Aumenta velocità avanti
   A : Gira a SX   S : Riduci velocità / Retromarcia   D : Gira a DX

Spazio o X : Azzera i motori (Fermo)
CTRL+C     : Esci dal programma

Nota: La manetta rimane impostata anche se rilasci il tasto!
--------------------------------------------------
"""

# Configurazione della spinta (Bilanciata per la fisica di Gazebo)
MAX_THRUST = 30000.0     # Impostiamo un limite reale sensato rispetto a prima
BACKWARD_THRUST = 10000.0 
THRUST_STEP = 500.0     # Incremento della spinta lineare (W/S)
TURN_STEP = 1000.0       # Intensità della sterzata (A/D)

def get_key(settings):
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
        
        self.pub_left = self.create_publisher(Float64, '/model/vessel_v2/joint/left_engine_joint/cmd_thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/model/vessel_v2/joint/right_engine_joint/cmd_thrust', 10)
        
        # Gestiamo due stati separati: spinta lineare e offset di virata
        self.linear_val = 0.0
        self.angular_val = 0.0
        
        self.get_logger().info("Motori pronti! Usa WASD per navigare.")

    def update_thrust(self, key):
        # W: Aumenta la spinta in avanti
        if key == 'w' or key == 'W':
            self.linear_val = min(self.linear_val + THRUST_STEP, MAX_THRUST)
            
        # S: Diminuisce la spinta avanti / va in retromarcia
        elif key == 's' or key == 'S':
            self.linear_val = max(self.linear_val - THRUST_STEP, -BACKWARD_THRUST)
            
        # A: Modifica l'asse per curvare a Sinistra (Motore DX spinge di più, SX di meno)
        elif key == 'a' or key == 'A':
            self.angular_val = min(self.angular_val + TURN_STEP, MAX_THRUST)
            
        # D: Modifica l'asse per curvare a Destra (Motore SX spinge di più, DX di meno)
        elif key == 'd' or key == 'D':
            self.angular_val = max(self.angular_val - TURN_STEP, -MAX_THRUST)
            
        # Stop d'emergenza (X o Spazio)
        elif key == ' ' or key == 'x' or key == 'X':
            self.linear_val = 0.0
            self.angular_val = 0.0
            
        else:
            # Se premi altri tasti (o rilasci), riduciamo gradualmente l'angolo di sterzata 
            # per far tornare la barca dritta, mantenendo però la velocità lineare (Manetta aperta)
            self.angular_val *= 0.5
            if abs(self.angular_val) < 10.0:
                self.angular_val = 0.0

        # Mixaggio della spinta differenziale
        thrust_left = self.linear_val + self.angular_val
        thrust_right = self.linear_val - self.angular_val
        total_thrust = thrust_left + thrust_right

        # Clip finale di sicurezza per evitare di superare i limiti fisici del motore
        thrust_left = max(min(thrust_left, MAX_THRUST), -MAX_THRUST)
        thrust_right = max(min(thrust_right, MAX_THRUST), -MAX_THRUST)

        # Invio comandi a Gazebo
        msg_l = Float64()
        msg_l.data = thrust_left
        msg_r = Float64()
        msg_r.data = thrust_right
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)
        
        # Output di debug pulito
        print(f"Manetta: {total_thrust:7.1f} N | Motore SX: {thrust_left:7.1f} N | Motore DX: {thrust_right:7.1f} N   ", end="\r")

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = TwinEngineTeleop()
    print(msg)
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            # Chiamiamo l'update sempre, così se non premi nulla la sterzata torna al centro stabilizzandosi
            node.update_thrust(key)
            if key == '\x03': # CTRL+C
                break
    except Exception as e:
        print(f"\nErrore: {e}")
    finally:
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.pub_left.publish(stop_msg)
        node.pub_right.publish(stop_msg)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()