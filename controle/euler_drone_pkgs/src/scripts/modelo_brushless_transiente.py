import numpy as np
import matplotlib.pyplot as plt

'CONSTANTES MOTOR'
KV_rpm = 400                  # [rpm/Vin]
kV_rad = KV_rpm * np.pi/30  # [rad/s/Vin]
Kt = 1/ kV_rad                # [Nm/A]
R = 0.116                       # [ohms]
L = 0.004                       # [L]
Vin = 19.47                     # [Vin]

'CONSTANTES HELICE'

Iz = 2*5.23*10**-3   #[kg*m²]
a = 0.00000153635  # coef. quadratico torque
b = -0.000198845   # Coef. linear torque

dt = 0.001  # [s]
tempo = 10 # [s]

# Funções para as EDOs
def dwdt(omega, i):
    return (Kt * i - a * omega**2 + b * omega) / Iz

def didt(omega, i):
    Vemf = omega / kV_rad
    return (Vin - R * i - Vemf) / L

# Condições iniciais
t = 0
omega = 0
i = 0

# Arrays para armazenar os resultados
t_vetor = []
omega_vetor = []
i_vetor = []


for step in range(tempo/dt):
    t_vetor.append(t)
    omega_vetor.append(omega)
    i_vetor.append(i)
    
    # Calculando os incrementos k1, k2, k3, k4 para omega
    k1_omega = dwdt(omega, i) * dt
    k1_i = didt(omega, i) * dt
    
    k2_omega = dwdt(omega + 0.5 * k1_omega, i + 0.5 * k1_i) * dt
    k2_i = didt(omega + 0.5 * k1_omega, i + 0.5 * k1_i) * dt
    
    k3_omega = dwdt(omega + 0.5 * k2_omega, i + 0.5 * k2_i) * dt
    k3_i = didt(omega + 0.5 * k2_omega, i + 0.5 * k2_i) * dt
    
    k4_omega = dwdt(omega + k3_omega, i + k3_i) * dt
    k4_i = didt(omega + k3_omega, i + k3_i) * dt
    
    # Atualizando os Vinalores de omega e i
    omega += (k1_omega + 2*k2_omega + 2*k3_omega + k4_omega) / 6
    i += (k1_i + 2*k2_i + 2*k3_i + k4_i) / 6
    
    # Atualizando o tempo
    t += dt

# Plotando os resultados
plt.plot(t_vetor, omega_vetor, label='Vinelocidade Angular')
plt.title(label= 'R = ' + str(R) + ', L = ' +  str(L) + ', Iz = ' + str(Iz) )
plt.plot(t_vetor, i_vetor, label='Corrente')
plt.xlabel('Tempo (s)')
plt.ylabel('Vinalor')
plt.legend()
plt.show()
print("omega = ",omega_vetor[-1])
print("corrente = ",i_vetor[-1])
