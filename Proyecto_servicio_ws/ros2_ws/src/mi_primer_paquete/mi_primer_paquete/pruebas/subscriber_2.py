import rclpy                          # Librería principal de ROS 2
from rclpy.node import Node           # Clase base para nodos
from std_msgs.msg import String       # Tipo de mensaje (texto)

class SubscriberNode(Node):           # Definimos un nodo suscriptor

    def __init__(self):
        super().__init__('subscriber_2')   # Inicializa el nodo con nombre
        
        self.subscription = self.create_subscription(
            String,                   # Tipo de mensaje que va a recibir
            'chatter_modificado',                # Tópico al que se suscribe
            self.listener_callback,   # Función que se ejecuta al recibir datos
            10                        # Tamaño de cola
        )

    def listener_callback(self, msg): # Esta función se ejecuta cuando llega un mensaje
        self.get_logger().info(
            f'Recibido: {msg.data}'   # Imprime el contenido del mensaje
        )


def main(args=None):
    rclpy.init(args=args)             # Inicializa ROS 2
    node = SubscriberNode()           # Crea el nodo
    rclpy.spin(node)                  # Mantiene el nodo activo esperando mensajes
    node.destroy_node()               # Destruye el nodo al terminar
    rclpy.shutdown()  