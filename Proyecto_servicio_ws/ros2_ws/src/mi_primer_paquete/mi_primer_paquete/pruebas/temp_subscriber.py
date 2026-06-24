import rclpy                          # Librería principal de ROS 2
from rclpy.node import Node           # Clase base para nodos
from std_msgs.msg import Float32      # Tipo de mensaje numérico

class TemperatureSubscriber(Node):    # Nodo que recibe temperatura

    def __init__(self):
        super().__init__('temp_subscriber')  # Nombre del nodo

        # Crear suscriptor al tópico 'temperature'
        self.subscription = self.create_subscription(
            Float32,                 # Tipo de mensaje esperado
            'temperature',           # Tópico al que se suscribe
            self.listener_callback,  # Función que se ejecuta al recibir datos
            10                       # Tamaño de cola
        )

    def listener_callback(self, msg):
        temperatura = msg.data       # Extraer valor recibido

        # Mostrar el valor recibido
        self.get_logger().info(f'Temperatura recibida: {temperatura:.2f} °C')

        # Lógica simple (decisión)
        if temperatura > 30.0:
            self.get_logger().warn('⚠️ Temperatura ALTA')
        else:
            self.get_logger().info('Temperatura normal')


def main(args=None):
    rclpy.init(args=args)            # Inicializa ROS 2
    node = TemperatureSubscriber()   # Crea el nodo
    rclpy.spin(node)                 # Mantiene el nodo activo
    node.destroy_node()              # Destruye el nodo
    rclpy.shutdown()                 # Cierra ROS 2