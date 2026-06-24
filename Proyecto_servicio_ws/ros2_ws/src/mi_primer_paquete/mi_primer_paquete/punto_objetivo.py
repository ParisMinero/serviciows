import rclpy                          # Librería principal de ROS 2
from rclpy.node import Node           # Clase base para nodos
from geometry_msgs.msg import PointStamped       # Tipo de mensaje (texto)

class PuntoObjetivoNode(Node):           # Definimos un nodo suscriptor

    def __init__(self):
        super().__init__('punto_objetivo')   # Inicializa el nodo con nombre
        
        self.subscription = self.create_subscription(
            PointStamped,                   # Tipo de mensaje que va a recibir
            'clicked_point',                # Tópico al que se suscribe
            self.listener_callback,   # Función que se ejecuta al recibir datos
            10                        # Tamaño de cola
        )

    def listener_callback(self, msg): # Esta función se ejecuta cuando llega un mensaje
        self.get_logger().info(
            f'Recibido: {msg.point} \n'   # Imprime el contenido del mensaje
            f'Coordenada en X: {msg.point.x} \n' 
            f'Coordenada en Y: {msg.point.y} \n' 
            f'Coordenada en Z: {msg.point.z} \n' 
        )
        



def main(args=None):
    rclpy.init(args=args)             # Inicializa ROS 2
    node = PuntoObjetivoNode()           # Crea el nodo
    rclpy.spin(node)                  # Mantiene el nodo activo esperando mensajes
    node.destroy_node()               # Destruye el nodo al terminar
    rclpy.shutdown()                  # Cierra ROS 2

main()