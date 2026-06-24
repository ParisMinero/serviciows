import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModifierNode(Node):

    def __init__(self):
        super().__init__('subscriber_node_modificado')

        # Suscriptor: escucha el tópico original
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        # Publicador: publica en otro tópico
        self.publisher_ = self.create_publisher(
            String,
            'chatter_modificado',
            10
        )

    def listener_callback(self, msg):
        # Crear nuevo mensaje
        new_msg = String()

        # Modificar el contenido recibido
        new_msg.data = msg.data + ' - mensaje modificado'

        # Publicar el mensaje modificado
        self.publisher_.publish(new_msg)

        # Mostrar en consola
        self.get_logger().info(f'Recibido: {msg.data}')
        self.get_logger().info(f'Publicado: {new_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ModifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()