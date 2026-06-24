import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped  
import numpy as np
import matplotlib.pyplot as plt
import sympy
from sympy import Matrix, sin, cos, diff, solve, symbols

class ModifierNode(Node):

    def __init__(self):
        super().__init__('de_clicked_a_cine')

        # Suscriptor: escucha el tópico original
        self.subscription = self.create_subscription(
            PointStamped,                   # Tipo de mensaje que va a recibir
            'clicked_point',                # Tópico al que se suscribe
            self.listener_callback,   # Función que se ejecuta al recibir datos
            10 
        )

        # Publicador: publica en otro tópico
        self.publisher_ = self.create_publisher(
            PointStamped,
            'chatter_modificado',
            10
        )

        

    def listener_callback(self, msg):
        alpha, beta, gamma, theta = symbols("alpha, beta, gamma, theta")
        z01, y12, x23, y34, z34, z4p, x34 = symbols("z_1__0, y_2__1, x_3__2, y_4__3, z_4__3, z_p__4, x_4__3")
        alpha_dot, beta_dot, gamma_dot, theta_dot = symbols("alpha_dot, beta_dot, gamma_dot, theta_dot")
        x_dot, y_dot, z_dot = symbols("x_dot, y_dot, z_dot")
        a_0, a_1, a_2, a_3, a_4, a_5, t = symbols("a_0, a_1, a_2, a_3, a_4, a_5, t")

        constantes = {z01: 0.057, y12: 0.043, x23: 0.12, x34: 0.1357, y34: 0.005624, z4p: 0.061823}

        T_0_p = Matrix([[sin(alpha)*sin(theta)-cos(alpha)*cos(theta)*cos(beta+gamma), sin(alpha)*cos(theta)+sin(theta)*cos(alpha)*cos(beta+gamma), sin(beta+gamma)*cos(alpha), x23*sin(beta)*cos(alpha)+x34*sin(beta+gamma)*cos(alpha)-y34*sin(alpha)+z4p*sin(beta+gamma)*cos(alpha)], 
                        [-sin(alpha)*cos(theta)*cos(beta+gamma)-sin(theta)*cos(alpha), sin(alpha)*sin(theta)*cos(beta+gamma)-cos(alpha)*cos(theta), sin(alpha)*sin(beta+gamma), x23*sin(alpha)*sin(beta)+x34*sin(alpha)*sin(beta+gamma)+y34*cos(alpha)+z4p*sin(alpha)*sin(beta+gamma)], 
                        [sin(beta+gamma)*cos(theta), -sin(theta)*sin(beta+gamma),  cos(beta+gamma), x23*cos(beta)+x34*cos(beta+gamma)+y12+z01+z4p*cos(beta+gamma)],
                        [0, 0, 0, 1]])

        xi = Matrix([T_0_p[0, 3],
                    T_0_p[1, 3],
                    T_0_p[2, 3]]).subs(constantes)

        J = Matrix([[-x23*sin(alpha)*sin(beta)-x34*sin(alpha)*sin(beta+gamma)-y34*cos(alpha)-z4p*sin(alpha)*sin(beta+gamma), (x23*cos(beta)+x34*cos(beta+gamma)+z4p*cos(beta+gamma))*cos(alpha), (x34+z4p)*cos(alpha)*cos(beta+gamma)], 
                    [x23*sin(beta)*cos(alpha)+x34*sin(beta+gamma)*cos(alpha)-y34*sin(alpha)+z4p*sin(beta+gamma)*cos(alpha), (x23*cos(beta)+x34*cos(beta+gamma)+z4p*cos(beta+gamma))*sin(alpha), (x34+z4p)*sin(alpha)*cos(beta+gamma)], 
                    [0, -x23*sin(beta)-x34*sin(beta+gamma)-z4p*sin(beta+gamma), (-x34-z4p)*sin(beta+gamma)]]).subs(constantes)

        J_inv = Matrix([[-sin(alpha)/(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma)), cos(alpha)/(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma)), 0], 
                        [(x23*sin(beta)*cos(alpha)+x34*sin(beta+gamma)*cos(alpha)-y34*sin(alpha)+z4p*sin(beta+gamma)*cos(alpha))*sin(beta+gamma)/(x23*(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma))*sin(gamma)), (x23*sin(alpha)*sin(beta)+x34*sin(alpha)*sin(beta+gamma)+y34*cos(alpha)+z4p*sin(alpha)*sin(beta+gamma))*sin(beta+gamma)/(x23*(x23*sin(beta)+x34*sin(beta+gamma)+z4p*sin(beta+gamma))*sin(gamma)), cos(beta+gamma)/(x23*sin(gamma))], 
                        [(-x23*sin(beta)*cos(alpha)-x34*sin(beta+gamma)*cos(alpha)+y34*sin(alpha)-z4p*sin(beta+gamma)*cos(alpha))/(x23*(x34+z4p)*sin(gamma)), (x23*sin(alpha)*sin(beta)+x34*sin(alpha)*sin(beta+gamma)+y34*cos(alpha)+z4p*sin(alpha)*sin(beta+gamma))/-(x23*(x34+z4p)*sin(gamma)), (x23*cos(beta)+x34*cos(beta+gamma)+z4p*cos(beta+gamma))/-(x23*(x34+z4p)*sin(gamma))]]).subs(constantes)

        # Vector de velocidades del efector final
        xi_dot = Matrix([x_dot, y_dot, z_dot])

        # Vector de velocidad en las juntas = jacobiano invero * Velocidad de la trayectoria (trayectoria)
        th_dot = (J_inv * xi_dot).subs(constantes)


        #tiempo en el que haremos el muestreo y movimiento 
        tf = 2
        # Construir una trayectoria con el spline de 5° orden
        lam = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
        lam_dot = diff(lam, t)
        lam_dot_dot = diff(lam_dot, t)

        eq1 = lam.subs({t: 0})
        eq2 = lam.subs({t: tf}) - 1
        eq3 = lam_dot.subs({t: 0})
        eq4 = lam_dot.subs({t: tf})
        eq5 = lam_dot_dot.subs({t: 0})
        eq6 = lam_dot_dot.subs({t: tf})

        solutions = solve((eq1, eq2, eq3, eq4, eq5, eq6),
                        (a_0, a_1, a_2, a_3, a_4, a_5))
        lam_s = lam.subs(solutions)

        # Posiciones iniciales de las juntas y efector final
        # Posicion inicial de las juntas (rad)
        alpha_in = 0.01
        beta_in = 0.01
        gamma_in = 0.01

        # Posición del efector final substituyendo en la postura (m, rad)
        x_in = xi[0].subs({alpha: alpha_in, beta: beta_in, gamma: gamma_in})
        y_in = xi[1].subs({alpha: alpha_in, beta: beta_in, gamma: gamma_in})
        z_in = xi[2].subs({alpha: alpha_in, beta: beta_in, gamma: gamma_in})

        # Posición final deseada (m, rad)
        x_f = msg.point.x
        y_f = msg.point.y
        z_f = msg.point.z


        # Posición, velocidad y aceleración en x
        x_eq = x_in + lam_s * (x_f - x_in)
        x_dot_eq = diff(x_eq, t)
        x_dot_dot_eq = diff(x_dot_eq, t)

        # Posición, velocidad y aceleración en y
        y_eq = y_in + lam_s * (y_f - y_in)
        y_dot_eq = diff(y_eq, t)
        y_dot_dot_eq = diff(y_dot_eq, t)

        # Posición, velocidad y aceleración en z
        z_eq = z_in + lam_s * (z_f - z_in)
        z_dot_eq = diff(z_eq, t)
        z_dot_dot_eq = diff(z_dot_eq, t)

        # Para manejarlo de forma discreta, muestreamos
        frec = 60
        dt = 1.0/frec
        dt
        samples = int(frec*tf) + 1
        samples

        # Generar arreglos para guardar muestreo
        t_m = Matrix.zeros(1, samples)
        for i in range(samples):
        t_m[i] = dt * i
        t_m

        # Arreglos para posición, velocidad y aceleración
        xi_m         = Matrix.zeros(3, samples)
        xi_dot_m     = Matrix.zeros(3, samples)
        xi_dot_dot_m = Matrix.zeros(3, samples)
        xi_m

        xi_t = Matrix([x_eq, y_eq, z_eq])
        xi_dot_t = Matrix([x_dot_eq, y_dot_eq, z_dot_eq])
        xi_dot_dot_t = Matrix([x_dot_dot_eq, y_dot_dot_eq, z_dot_dot_eq])
        xi_t

        # Muestreo
        for i in range(samples):
        xi_m[:, i]         = xi_t.        subs({t: t_m[i]})
        xi_dot_m[:, i]     = xi_dot_t.    subs({t: t_m[i]})
        xi_dot_dot_m[:, i] = xi_dot_dot_t.subs({t: t_m[i]})
        xi_m

        #Arreglos para posición y velocidad de las juntas
        th_m = Matrix.zeros(3, samples)
        th_dot_m = Matrix.zeros(3, samples)
        th_dot_dot_m = Matrix.zeros(3, samples)
        th_m[:, 0] = Matrix([alpha_in, beta_in, gamma_in])
        # Cinemática inversa
        for i in range(samples):
        th_dot_m[:, i] = (th_dot.subs({alpha: th_m[0, i], beta: th_m[1, i], 
                                        gamma: th_m[2, i], x_dot: xi_dot_m[0, i],
                                        y_dot: xi_dot_m[1, i], 
                                        z_dot: xi_dot_m[2, i]})).evalf()
        if i < samples - 1:
            th_m[:, i+1] = th_m[:, i] + th_dot_m[:, i] * dt
        if i > 0:
            th_dot_dot_m[:, i-1] = (th_dot_m[:, i] - th_dot_m[:, i-1])/dt
        th_dot_dot_m


        # print(f"Alpha final: {th_m[0, -1]}") // Pasos
        # print(f"Beta final:  {th_m[1, -1]}") // Brazo
        # print(f"Gamma final: {th_m[2, -1]}") // Antebrazo


def main(args=None):
    rclpy.init(args=args)
    node = ModifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()