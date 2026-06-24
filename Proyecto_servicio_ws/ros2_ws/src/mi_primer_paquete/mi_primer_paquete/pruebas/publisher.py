import rclpy                          # Librería principal de ROS 2 en Python
from rclpy.node import Node           # Clase base para crear nodos
from std_msgs.msg import String       # Tipo de mensaje de texto

class PublisherNode(Node):            # Creamos una clase que hereda de Node

    def __init__(self):               # Constructor: se ejecuta al crear el nodo
        super().__init__('publisher_node')   # Inicializa el nodo con ese nombre
        
        self.publisher_ = self.create_publisher(
            String,                   # Tipo de mensaje
            'chatter',                # Nombre del tópico
            10                        # Tamaño de cola
        )
        
        self.timer = self.create_timer(
            5.0,                      # Cada 1 segundo
            self.publish_message      # Llama a esta función
        )
        
        self.count = 0                # Contador para cambiar el mensaje

    def publish_message(self):        # Función que se ejecuta cada segundo
        msg = String()                # Crear mensaje tipo String
        msg.data = f'Hola {self.count}'  # Contenido del mensaje
        self.publisher_.publish(msg)  # Publicar en el tópico
        self.get_logger().info(f'Publicando: {msg.data}')  # Mostrar en consola
        self.count += 1               # Aumentar contador


def main(args=None):
    rclpy.init(args=args)             # Iniciar ROS 2
    node = PublisherNode()            # Crear el nodo
    rclpy.spin(node)                  # Mantenerlo corriendo
    node.destroy_node()               # Destruir nodo al salir
    rclpy.shutdown()                  # Cerrar ROS 2