import matplotlib.pyplot as plt
import numpy as np
import csv

import matplotlib as mpl

fig, ax = plt.subplots()  # una figura con un solo eje
ax.set_xlabel('posición del robot')
ax.set_ylabel('velocidad')
ax.set_title('Posición del robot vs velocidad')

# Cargar datos de los archivos CSV
# data3_1 = np.loadtxt('data3_1.csv', delimiter=',')
# data3_2 = np.loadtxt('data3_2.csv', delimiter=',')
# data3_3 = np.loadtxt('data3_3.csv', delimiter=',')

# Graficar datos de los archivos CSV
# ax.plot(data3_1[:,1], data3_1[:,2], label='Datos 3_1')
# ax.plot(data3_2[:,1], data3_2[:,2], label='Datos 3_2')
# ax.plot(data3_3[:,1], data3_3[:,2], label='Datos 3_3')

# Cargar y calcular datos del archivo CSV "data4.csv"
data4 = np.loadtxt('data4.csv', delimiter=',')
vels4 = data4[:,2]
ang = data4[:,11]/10
mse = np.square(np.subtract(vels4, 2)).mean()
rms4 = np.sqrt(mse)
print("rms 4:", rms4)

# vels3 = data3_3[:,2]
# mse = np.square(np.subtract(vels3, 2)).mean()
# rms3 = np.sqrt(mse)
# print("rms 3:", rms3)

# Graficar datos del archivo "data4.csv"
ax.plot(data4[:,1], data4[:,2], label='Datos 4')
# ax.plot(data4[:,1], ang, label='Ángulo')

# Mostrar la leyenda
ax.legend()

plt.savefig('Fase4.png')
plt.show()
