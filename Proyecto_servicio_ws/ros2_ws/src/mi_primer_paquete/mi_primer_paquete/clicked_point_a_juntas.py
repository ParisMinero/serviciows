#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from sympy import Matrix, sin, cos, diff, solve, symbols
import time

class ClickedPointAJuntas(Node):

    def __init__(self):
        super().__init__('clicked_point_node')

        # Suscriptor al punto que marcas en RViz
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10
        )

        # Publicador de estados de juntas para RViz
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Estado actual de las juntas
        self.posiciones = [0.01, 0.01, 0.01]

        self.nombres_juntas = [
            'joint_base_hombro',
            'joint_hombro_brazo',
            'joint_brazo_antebrazo'
        ]

        self.get_logger().info('Nodo listo.')
        self.get_logger().info('Haz clic en RViz con la herramienta "Publish Point".')

        # Publicar continuamente para mantener actualizado RViz
        self.timer = self.create_timer(0.1, self.publicar_estado)

    def publicar_estado(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.nombres_juntas
        msg.position = self.posiciones
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Punto recibido: x={msg.point.x:.4f}, y={msg.point.y:.4f}, z={msg.point.z:.4f}'
        )

        try:
            # Variables simbólicas
            alpha, beta, gamma, theta = symbols("alpha beta gamma theta")
            z01, y12, x23, y34, z4p, x34 = symbols("z01 y12 x23 y34 z4p x34")
            x_dot, y_dot, z_dot = symbols("x_dot y_dot z_dot")
            a_0, a_1, a_2, a_3, a_4, a_5, t = symbols("a_0 a_1 a_2 a_3 a_4 a_5 t")

            # Constantes geométricas
            constantes = {
                z01: 0.057,
                y12: 0.043,
                x23: 0.12,
                x34: 0.1357,
                y34: 0.005624,
                z4p: 0.061823
            }

            # Cinemática directa del efector final
            T_0_p = Matrix([
                [
                    sin(alpha)*sin(theta) - cos(alpha)*cos(theta)*cos(beta+gamma),
                    sin(alpha)*cos(theta) + sin(theta)*cos(alpha)*cos(beta+gamma),
                    sin(beta+gamma)*cos(alpha),
                    x23*sin(beta)*cos(alpha) + x34*sin(beta+gamma)*cos(alpha)
                    - y34*sin(alpha) + z4p*sin(beta+gamma)*cos(alpha)
                ],
                [
                    -sin(alpha)*cos(theta)*cos(beta+gamma) - sin(theta)*cos(alpha),
                    sin(alpha)*sin(theta)*cos(beta+gamma) - cos(alpha)*cos(theta),
                    sin(alpha)*sin(beta+gamma),
                    x23*sin(alpha)*sin(beta) + x34*sin(alpha)*sin(beta+gamma)
                    + y34*cos(alpha) + z4p*sin(alpha)*sin(beta+gamma)
                ],
                [
                    sin(beta+gamma)*cos(theta),
                    -sin(theta)*sin(beta+gamma),
                    cos(beta+gamma),
                    x23*cos(beta) + x34*cos(beta+gamma) + y12 + z01 + z4p*cos(beta+gamma)
                ],
                [0, 0, 0, 1]
            ])

            xi = Matrix([
                T_0_p[0, 3],
                T_0_p[1, 3],
                T_0_p[2, 3]
            ]).subs(constantes)

            # Jacobiano inverso
            J_inv = Matrix([
                [
                    -sin(alpha)/(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma)),
                    cos(alpha)/(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma)),
                    0
                ],
                [
                    (x23*sin(beta)*cos(alpha)+x34*sin(beta+gamma)*cos(alpha)-y34*sin(alpha)+z4p*sin(beta+gamma)*cos(alpha))*sin(beta+gamma)
                    /(x23*(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma))*sin(gamma)),

                    (x23*sin(alpha)*sin(beta)+x34*sin(alpha)*sin(beta+gamma)+y34*cos(alpha)+z4p*sin(alpha)*sin(beta+gamma))*sin(beta+gamma)
                    /(x23*(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma))*sin(gamma)),

                    cos(beta+gamma)/(x23*sin(gamma))
                ],
                [
                    (-x23*sin(beta)*cos(alpha)-x34*sin(beta+gamma)*cos(alpha)+y34*sin(alpha)-z4p*sin(beta+gamma)*cos(alpha))
                    /(x23*(x34+z4p)*sin(gamma)),

                    -(x23*sin(alpha)*sin(beta)+x34*sin(alpha)*sin(beta+gamma)+y34*cos(alpha)+z4p*sin(alpha)*sin(beta+gamma))
                    /(x23*(x34+z4p)*sin(gamma)),

                    -(x23*cos(beta)+x34*cos(beta+gamma)+z4p*cos(beta+gamma))
                    /(x23*(x34+z4p)*sin(gamma))
                ]
            ]).subs(constantes)

            # Vector de velocidades cartesianas
            xi_dot = Matrix([x_dot, y_dot, z_dot])

            # Velocidades articulares
            th_dot = J_inv * xi_dot

            # Tiempo de trayectoria
            tf = 2.0

            # Polinomio de 5to orden
            lam = a_0 + a_1*t + a_2*t**2 + a_3*t**3 + a_4*t**4 + a_5*t**5
            lam_dot = diff(lam, t)
            lam_dot_dot = diff(lam_dot, t)

            eq1 = lam.subs({t: 0})
            eq2 = lam.subs({t: tf}) - 1
            eq3 = lam_dot.subs({t: 0})
            eq4 = lam_dot.subs({t: tf})
            eq5 = lam_dot_dot.subs({t: 0})
            eq6 = lam_dot_dot.subs({t: tf})

            solutions = solve(
                (eq1, eq2, eq3, eq4, eq5, eq6),
                (a_0, a_1, a_2, a_3, a_4, a_5)
            )

            lam_s = lam.subs(solutions)

            # Postura inicial
            alpha_in = float(self.posiciones[0])
            beta_in = float(self.posiciones[1])
            gamma_in = float(self.posiciones[2])

            # Posición inicial del efector
            x_in = xi[0].subs({alpha: alpha_in, beta: beta_in, gamma: gamma_in})
            y_in = xi[1].subs({alpha: alpha_in, beta: beta_in, gamma: gamma_in})
            z_in = xi[2].subs({alpha: alpha_in, beta: beta_in, gamma: gamma_in})

            # Punto final deseado desde RViz
            x_f = msg.point.x
            y_f = msg.point.y
            z_f = msg.point.z

            # Trayectoria cartesiana
            x_eq = x_in + lam_s * (x_f - x_in)
            y_eq = y_in + lam_s * (y_f - y_in)
            z_eq = z_in + lam_s * (z_f - z_in)

            x_dot_eq = diff(x_eq, t)
            y_dot_eq = diff(y_eq, t)
            z_dot_eq = diff(z_eq, t)

            # Muestreo
            frec = 60
            dt = 1.0 / frec
            samples = int(frec * tf) + 1

            # Arreglos
            xi_m = Matrix.zeros(3, samples)
            xi_dot_m = Matrix.zeros(3, samples)

            xi_t = Matrix([x_eq, y_eq, z_eq])
            xi_dot_t = Matrix([x_dot_eq, y_dot_eq, z_dot_eq])

            for i in range(samples):
                tiempo_i = dt * i
                xi_m[:, i] = xi_t.subs({t: tiempo_i})
                xi_dot_m[:, i] = xi_dot_t.subs({t: tiempo_i})

            # Arreglos articulares
            th_m = Matrix.zeros(3, samples)
            th_dot_m = Matrix.zeros(3, samples)

            th_m[:, 0] = Matrix([alpha_in, beta_in, gamma_in])

            # Integración de velocidades articulares
            for i in range(samples):
                th_dot_m[:, i] = (
                    th_dot.subs({
                        alpha: th_m[0, i],
                        beta: th_m[1, i],
                        gamma: th_m[2, i],
                        x_dot: xi_dot_m[0, i],
                        y_dot: xi_dot_m[1, i],
                        z_dot: xi_dot_m[2, i]
                    })
                ).evalf()

                if i < samples - 1:
                    th_m[:, i + 1] = th_m[:, i] + th_dot_m[:, i] * dt

            # Tomar el último valor calculado
            alpha_final = float(th_m[0, samples - 1])
            beta_final = float(th_m[1, samples - 1])
            gamma_final = float(th_m[2, samples - 1])

            # Límites del URDF
            limites = [
                (-3.14, 3.14),
                (-0.85, 1.9),
                (-1.57, 1.87)
            ]

            valores_finales = [alpha_final, beta_final, gamma_final]

            # Validar
            for i, valor in enumerate(valores_finales):
                minimo, maximo = limites[i]
                if valor < minimo or valor > maximo:
                    self.get_logger().warn(
                        f'La junta {self.nombres_juntas[i]} quedó fuera de rango: '
                        f'{valor:.4f} rad. Rango permitido: [{minimo}, {maximo}]'
                    )
                    return

            # Guardar nueva postura
            for i in range(samples):
                self.posiciones = [
                    float(th_m[0, i]),
                    float(th_m[1, i]),
                    float(th_m[2, i])
                ]

                self.publicar_estado()
                time.sleep(dt)

            self.get_logger().info(
                f'Juntas calculadas: '
                f'alpha={alpha_final:.4f}, beta={beta_final:.4f}, gamma={gamma_final:.4f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error en el cálculo: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointAJuntas()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()