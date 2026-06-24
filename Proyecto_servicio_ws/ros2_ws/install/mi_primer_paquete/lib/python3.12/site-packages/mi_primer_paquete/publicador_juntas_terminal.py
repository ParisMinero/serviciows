#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import math


class PublicadorJuntasTerminal(Node):
    def __init__(self):
        super().__init__('publicador_juntas_terminal')

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Estado actual de las 3 juntas
        self.posiciones = [0.0, 0.0, 0.0]

        # Nombres EXACTOS de las juntas del URDF
        self.nombres_juntas = [
            'joint_base_hombro',
            'joint_hombro_brazo',
            'joint_brazo_antebrazo'
        ]

        # Publica continuamente el último estado para que RViz lo mantenga actualizado
        self.timer = self.create_timer(0.1, self.publicar_estado)

        self.get_logger().info('Nodo iniciado.')
        self.get_logger().info('Escribe 3 valores separados por espacio y presiona Enter.')
        self.get_logger().info('Ejemplo: 0 0 3.14')
        self.get_logger().info('También puedes escribir: salir')

        # Hilo aparte para no bloquear ROS mientras esperas input en terminal
        self.hilo_entrada = threading.Thread(target=self.leer_terminal, daemon=True)
        self.hilo_entrada.start()

    def publicar_estado(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.nombres_juntas
        msg.position = self.posiciones
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)

    def leer_terminal(self):
        while rclpy.ok():
            try:
                entrada = input(
                    '\nIngresa q1 q2 q3 en radianes '
                    '(o "deg a b c" en grados): '
                ).strip()

                if entrada.lower() == 'salir':
                    self.get_logger().info('Cerrando nodo...')
                    rclpy.shutdown()
                    break

                if not entrada:
                    continue

                partes = entrada.split()

                # Modo grados: deg 0 45 90
                if partes[0].lower() == 'deg':
                    if len(partes) != 4:
                        print('Formato incorrecto. Usa: deg 0 45 90')
                        continue

                    q1 = math.radians(float(partes[1]))
                    q2 = math.radians(float(partes[2]))
                    q3 = math.radians(float(partes[3]))
                else:
                    # Modo radianes: 0 0 3.14
                    if len(partes) != 3:
                        print('Formato incorrecto. Usa: 0 0 3.14')
                        continue

                    q1 = float(partes[0])
                    q2 = float(partes[1])
                    q3 = float(partes[2])

                # Límites de tu URDF
                limites = [
                    (-3.14, 3.14),   # joint_base_hombro
                    (-0.85, 1.9),    # joint_hombro_brazo
                    (-1.57, 1.87)    # joint_brazo_antebrazo
                ]

                valores = [q1, q2, q3]

                # Validar límites
                fuera_de_rango = False
                for i, valor in enumerate(valores):
                    minimo, maximo = limites[i]
                    if valor < minimo or valor > maximo:
                        print(
                            f'La junta {self.nombres_juntas[i]} está fuera de rango: '
                            f'{valor:.4f} rad. '
                            f'Rango permitido: [{minimo}, {maximo}]'
                        )
                        fuera_de_rango = True

                if fuera_de_rango:
                    continue

                # Actualizar posiciones
                self.posiciones = valores

                print('\nNuevas posiciones enviadas:')
                print(f'  {self.nombres_juntas[0]} = {self.posiciones[0]:.4f} rad')
                print(f'  {self.nombres_juntas[1]} = {self.posiciones[1]:.4f} rad')
                print(f'  {self.nombres_juntas[2]} = {self.posiciones[2]:.4f} rad')

            except ValueError:
                print('Error: escribe solo números válidos.')
            except EOFError:
                self.get_logger().info('Entrada finalizada. Cerrando nodo...')
                rclpy.shutdown()
                break
            except Exception as e:
                print(f'Error inesperado: {e}')


def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorJuntasTerminal()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()