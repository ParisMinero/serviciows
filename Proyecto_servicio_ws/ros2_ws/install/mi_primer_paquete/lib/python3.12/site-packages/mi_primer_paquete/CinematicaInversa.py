import sympy
from sympy import Matrix, sin, cos, diff, solve, symbols

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

xi = Matrix([T_0_p[0, 3],import rclpy
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
x_f = 0.317523
y_f = 0.005624
z_f = 0.1


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


print(f"Alpha final: {th_m[0, -1]}")
print(f"Beta final:  {th_m[1, -1]}")
print(f"Gamma final: {th_m[2, -1]}")