import rclpy                          # Librería principal de ROS 2
from rclpy.node import Node           # Clase base para nodos
from std_msgs.msg import Float32      # Tipo de mensaje numérico (decimal)

import random                         # Para generar valores aleatorios

class TemperaturePublisher(Node):     # Nodo que publica temperatura

    def __init__(self):
        super().__init__('temp_publisher')   # Nombre del nodo

        # Crear publicador al tópico 'temperature'
        self.publisher_ = self.create_publisher(
            Float32,                 # Tipo de mensaje
            'temperature',           # Nombre del tópico
            10                       # Tamaño de cola
        )

        # Crear temporizador (cada 1 segundo)
        self.timer = self.create_timer(
            1.0,                     # Intervalo en segundos
            self.publish_temperature
        )

    def publish_temperature(self):
        msg = Float32()              # Crear mensaje tipo Float32

        # Generar temperatura aleatoria entre 20 y 40 grados
        msg.data = random.uniform(20.0, 40.0)

        # Publicar el mensaje
        self.publisher_.publish(msg)

        # Mostrar en consola
        self.get_logger().info(f'Temperatura enviada: {msg.data:.2f} °C')


def main(args=None):
    rclpy.init(args=args)            # Inicializa ROS 2
    node = TemperaturePublisher()    # Crea el nodo
    rclpy.spin(node)                 # Mantiene el nodo activo
    node.destroy_node()              # Destruye el nodo al cerrar
    rclpy.shutdown()                 # Apaga ROS 2