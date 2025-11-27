import pickle
import matplotlib.pyplot as plt
import matplotlib as mpl


filepath = "/home/darwin/Documents/DARwInOP2_ws/src/EntrenamientoRL/scripts/estado_entrenamiento.pkl"
mpl.rcParams['text.usetex'] = True


# Cargar el estado del entrenamiento desde el archivo
def cargar_estado(archivo=filepath):
    with open(archivo, 'rb') as f:
        estado = pickle.load(f)
    return estado


# Cargar el historial de recompensas
archivo_estado = filepath
estado_cargado = cargar_estado(archivo_estado)
historial_recompensas = estado_cargado['historial_recompensas']

# Gráfico de las recompensas
plt.figure(figsize=(10, 6))
plt.plot(historial_recompensas, label='Recompensas por episodio')
plt.xlabel('Episodio')
plt.ylabel('Recompensa Total')
plt.title('Recompensas a lo Largo de los Episodios de Entrenamiento')
plt.legend()
plt.grid(True)
plt.show()
