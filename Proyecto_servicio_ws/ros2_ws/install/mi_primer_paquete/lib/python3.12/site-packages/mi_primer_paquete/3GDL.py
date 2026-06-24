import numpy as np
#from np.rad2deg
import matplotlib.pyplot as plt
#Parámetros de los eslabones

#L1 = Brazo, L2 = antebrazo

L1, L2, L3= 1.0, 1.0, 1.0



def Cin_Inv(x, y, z):

# Distancia al punto deseado (teorema de pitagoras)

    dist_sq = np.sqrt(x**2 + y**2 +(z-L1)**2)

# Ley de cosenos para theta3

    cos_theta3 = (dist_sq**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0) 

    theta3 = np.arctan((np.sqrt(1-cos_theta3)),cos_theta3)

# Cálculo de theta2

    theta2 = np.arctan2(z-L1, np.sqrt(x**2 +y**2)) - np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))

# Cálculo de theta1

    theta1=np.arctan(y,x)


    return theta1, theta2,theta3



#trayectoria de ejemplo

cant = int(input("Dame la cantidad de puntos"))

#x_path = 1.0 + 0.3 * np.cos(t)
#y_path = 1.0 + 0.3 * np.sin(t)
x_ini = float(input("Dame coordenada x inicial "))
y_ini = float(input("Dame coordenada y inicial "))
z_ini = float(input("Dame coordenada z inicial "))

x_fin = float(input("Dame coordenada x final"))
y_fin = float(input("Dame coordenada y final"))
z_fin = float(input("Dame coordenada z final"))

t = np.linspace(0, 1, cant)
#x_path = np.linspace(0.0001, x_fin, cant)
#y_path = np.linspace(0.0001, y_fin, cant)
s = 10*t**3 - 15*t**4 + 6*t**5
x_path = x_ini + (x_fin - x_ini) * s
y_path = y_ini + (y_fin - y_ini) * s
z_path = z_ini + (z_fin - z_ini) * s
#Cálculo de los ángulos

theta1, theta2, theta3 = Cin_Inv(x_path, y_path,z_path)

#print ("los valores de theta1 y theta 2 son:",theta1,theta2)

#Cálculo de velocidades y aceleraciones
dt = t[1] - t[0]

v_t1 = np.gradient(theta1, dt)
v_t2 = np.gradient(theta2, dt)
v_t3 = np.gradient(theta3, dt)
a_t1 = np.gradient(v_t1, dt)
a_t2 = np.gradient(v_t2, dt)
a_t3 = np.gradient(v_t3, dt)

#x_acum = np.cumsum(v_t1) * dt
#tint = x_acum + theta1[0]

tint1 = np.trapz(v_t1, dx=dt)
print(tint1*(180/np.pi))
tint2 = np.trapz(v_t2, dx=dt)
print(tint2*(180/np.pi))
tint3 = np.trapz(v_t3, dx=dt)
print(tint3*(180/np.pi))
tace1 = np.trapz(a_t1, dx=dt)
print(tace1*(180/np.pi))
tace2 = np.trapz(a_t2, dx=dt)
print(tace2*(180/np.pi))
tace3 = np.trapz(a_t3, dx=dt)
print(tace3*(180/np.pi))
#Graficación

fig, axs = plt.subplots(3, 1, figsize=(8, 10))
axs[0].plot(t, theta1*(180/np.pi), label='Theta 1'); axs[0].plot(t, theta2*(180/np.pi), label='Theta 2'); axs[0].plot(t, theta3*(180/np.pi), label='Theta 3'); axs[0].set_title('Posición Angular');axs[0].legend()
axs[1].plot(t, v_t1*(180/np.pi), label='Omega 1'); axs[1].plot(t, v_t2*(180/np.pi), label='Omega 2'); axs[1].plot(t, v_t3*(180/np.pi), label='Omega 3'); axs[1].set_title('Velocidad Angular');axs[1].legend()
axs[2].plot(t, a_t1*(180/np.pi), label='Alpha1'); axs[2].plot(t, a_t2*(180/np.pi), label='Alpha 2');  axs[2].plot(t, a_t3*(180/np.pi), label='Alpha 3');axs[2].set_title('Aceleración Angular');axs[2].legend()
plt.tight_layout()
plt.show()