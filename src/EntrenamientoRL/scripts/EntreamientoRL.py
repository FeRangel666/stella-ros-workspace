from op3_walking_module_msgs.msg import WalkingParam
from typing import List, Optional, Tuple
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import rospy
import pickle
import os
import signal
import sys


def manejar_interrupcion(signum, frame): # noqa
    """
    Maneja la interrupción del programa (Ctrl+C).

    Args:
        signum: Número de la señal recibida. No utilizado en esta función.
        frame: Frame actual en el momento de la señal. No utilizado en esta función.
    """
    print("Interrupción detectada. Publicando pose inicial y cerrando...")
    # Lógica para manejar la interrupción
    sys.exit(0)


rangos_aceleracion_x = np.linspace(-13, 13, 31)
rangos_aceleracion_y = np.linspace(-7, 7, 31)
rangos_aceleracion_z = np.linspace(-17, -3, 31)

# Rangos de pitch y roll
rangos_pitch = np.linspace(-30, 30, 41)  # De -30 a 30 grados
rangos_roll = np.linspace(-10, 10, 21)  # De -10 a 10 grados


def guardar_estado(agente, episodio_actual, recompensas, archivo='/home/darwin/Documents/DARwInOP2_ws/src/'
                                                                 'EntrenamientoRL/scripts/estado_entrenamiento.pkl'):
    informacion = {
        'q_tabla': agente.q_tabla,
        'epsilon': agente.epsilon,
        'alpha': agente.alpha,
        'gamma': agente.gamma,
        'episodio_actual': episodio_actual,
        'historial_recompensas': recompensas,
        # Puedes añadir más datos aquí si es necesario
    }
    with open(archivo, 'wb') as f:
        pickle.dump(informacion, f)


def cargar_estado(archivo='/home/darwin/Documents/DARwInOP2_ws/src/EntrenamientoRL/scripts/estado_entrenamiento.pkl'):
    with open(archivo, 'rb') as f:
        informacion = pickle.load(f)
    return informacion


def mapear_aceleracion_a_estado(valor_aceleracion, rangos):
    """
    Mapea un valor de aceleración a un estado discreto basado en rangos predefinidos.

    Args:
        valor_aceleracion (float): Valor de aceleración a mapear.
        rangos (List[float]): Lista de valores que definen los rangos de aceleración.

    Returns:
        int: Índice del rango en el que cae el valor de aceleración.
    """
    for i in range(len(rangos) - 1):
        if rangos[i] <= valor_aceleracion < rangos[i + 1]:
            return i
    return len(rangos) - 1


def mapear_estado_a_indice(estado_x, estado_y, estado_z):
    """
    Mapea un estado tridimensional de aceleraciones a un índice único en un espacio de estados.

    Args:
        estado_x (float): Valor de aceleración en el eje X.
        estado_y (float): Valor de aceleración en el eje Y.
        estado_z (float): Valor de aceleración en el eje Z.

    Returns:
        int: Índice único que representa el estado tridimensional en el espacio de estados.
    """
    indice_x = mapear_aceleracion_a_estado(estado_x, rangos_aceleracion_x)
    indice_y = mapear_aceleracion_a_estado(estado_y, rangos_aceleracion_y)
    indice_z = mapear_aceleracion_a_estado(estado_z, rangos_aceleracion_z)
    num_estados_y = len(rangos_aceleracion_y)
    num_estados_z = len(rangos_aceleracion_z)
    return indice_x * (num_estados_y * num_estados_z) + indice_y * num_estados_z + indice_z


def mapear_accion_a_indice(pitch, roll):
    """
    Mapea una combinación de valores de pitch y roll a un índice único en un espacio de acciones.

    Args:
        pitch (float): Valor de pitch a mapear.
        roll (float): Valor de roll a mapear.

    Returns:
        int: Índice único que representa la combinación de pitch y roll en el espacio de acciones.
    """
    indice_pitch = np.abs(rangos_pitch - pitch).argmin()
    indice_roll = np.abs(rangos_roll - roll).argmin()

    num_estados_roll = len(rangos_roll)
    return indice_pitch * num_estados_roll + indice_roll


class AgenteRL:
    """
    Clase para implementar un agente de aprendizaje por refuerzo.

    Attributes:
        q_tabla (np.ndarray): Tabla Q para el aprendizaje Q-learning.
        alpha (float): Tasa de aprendizaje.
        gamma (float): Factor de descuento.
        epsilon (float): Probabilidad de exploración.
    """

    def __init__(self, num_estados_clase, num_acciones_clase, alpha_clase, gamma_clase, epsilon_clase):
        self.q_tabla = np.zeros((num_estados_clase, num_acciones_clase))
        self.alpha = alpha_clase  # Tasa de aprendizaje
        self.gamma = gamma_clase  # Factor de descuento
        self.epsilon = epsilon_clase  # Probabilidad de exploración

        self.acel_x, self.acel_y, self.acel_z = 0, 0, 0
        self.recompensa = 0

    def seleccionar_accion(self, estado_clase):
        if np.random.rand() < self.epsilon:
            return np.random.choice(self.q_tabla.shape[1])
        else:
            return np.argmax(self.q_tabla[estado_clase])

    def actualizar_q_tabla(self, estado_clase, accion_clase, recompensa_clase, nuevo_estado_clase):
        mejor_valor_futuro = np.max(self.q_tabla[nuevo_estado_clase])
        valor_actual = self.q_tabla[estado_clase, accion_clase]
        self.q_tabla[estado_clase, accion_clase] = valor_actual + self.alpha * (
                recompensa_clase + self.gamma * mejor_valor_futuro - valor_actual)

    def es_final_del_episodio(self, lectura_actual: Tuple[float, float, float], rangos_x, rangos_y, rangos_z):
        """
        Determina si el episodio actual debe terminar basándose en la lectura actual del IMU.

        Args:
            lectura_actual (Tuple[float, float, float]): La última lectura del IMU en los ejes x, y, z.
            rangos_x (np.ndarray): Arreglo de Numpy con los rangos de aceleración para el eje X.
            rangos_y (np.ndarray): Arreglo de Numpy con los rangos de aceleración para el eje Y.
            rangos_z (np.ndarray): Arreglo de Numpy con los rangos de aceleración para el eje Z.

        Returns:
            tuple:
                - bool: Retorna True si el episodio debe terminar, False en caso contrario.
                - str: La razón por la que el episodio termina o "En rango" si no termina.
        """

        self.acel_x, self.acel_y, self.acel_z = lectura_actual

        # Comprobar si las aceleraciones están fuera de los rangos establecidos
        if self.acel_x < rangos_x[0]:
            return True, "Aceleración X menor que el rango mínimo"
        elif self.acel_x > rangos_x[-1]:
            return True, "Aceleración X mayor que el rango máximo"
        elif self.acel_y < rangos_y[0]:
            return True, "Aceleración Y menor que el rango mínimo"
        elif self.acel_y > rangos_y[-1]:
            return True, "Aceleración Y mayor que el rango máximo"
        elif self.acel_z < rangos_z[0]:
            return True, "Aceleración Z menor que el rango mínimo"
        elif self.acel_z > rangos_z[-1]:
            return True, "Aceleración Z mayor que el rango máximo"

        return False, "En rango"

    def calcular_recompensa(self, lectura_actual, termina_episodio, razon_terminacion, margen=0.1):
        acel_x, acel_y, acel_z = lectura_actual
        self.recompensa = 0

        # Recompensa por estabilidad
        if abs(acel_x) <= margen and abs(acel_y) <= margen and abs(acel_z + 9.81) <= margen:
            self.recompensa += 20  # Valor positivo para estabilidad

        # Penalización por inestabilidad o salirse de rangos
        else:
            distancia = max(abs(acel_x), abs(acel_y), abs(acel_z + 9.81)) - margen
            self.recompensa -= distancia * 30  # Penalización en función de la distancia al margen de estabilidad

        # Penalización grande si el episodio termina debido a una caída
        if termina_episodio and "Aceleración" in razon_terminacion:
            self.recompensa -= 10000  # Penalización significativa por caída

        return self.recompensa


class ControlRobot:
    """
    Clase para leer datos del IMU de un robot usando ROS, manteniendo solo la última lectura.
    """

    def __init__(self):
        """
        Inicializa el nodo ROS y los publicadores. Crea los publicadores necesarios para
        controlar las poses del robot, el módulo de caminata y los parámetros de la caminata.
        """
        rospy.init_node('Nodo_publisher_shido')
        rospy.loginfo("[PUBLISHER  PY  NODE] " + rospy.get_namespace())

        self.ini_pose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)
        self.module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
        self.walk = rospy.Publisher("/robotis/walking/command", String, queue_size=10)
        self.walk_param_set = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=10)

        self.caminata_activa = False

        # Subscripción al tópico del IMU
        self.imu_subscriber = rospy.Subscriber("/robotis/cm_740/imu", Imu, self.imu_callback)
        self.ultima_lectura: Optional[List[float]] = None

    def publicar(self, tipo_pose: str) -> None:
        """
        Publica la pose especificada por el tipo.

        :param tipo_pose: str
            Tipo de pose a publicar. Puede ser 'pose_inicial' para una pose inicial
            o 'pose_caminata' para activar el módulo de caminata.
        """
        if tipo_pose == 'pose_inicial':
            self.ini_pose.publish('ini_pose')
        elif tipo_pose == 'intrinseco':
            self.module.publish('none')
        elif tipo_pose == 'pose_caminata':
            self.module.publish('walking_module')
            self.establecer_parametros()
        else:
            rospy.loginfo("Tipo de pose no reconocido")

    def imu_callback(self, data: Imu):
        """
        Callback para recibir y almacenar la última lectura del IMU.

        :param data: Imu
            Mensaje IMU recibido desde el tópico.
        """
        self.ultima_lectura = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

    def obtener_ultima_lectura(self) -> Optional[List[float]]:
        """
        Obtiene la última lectura de aceleración si está disponible.

        :return: Optional[List[float]]
            Retorna la última lectura de aceleración (una lista de tres flotantes) o None si no hay datos disponibles.
        """
        return self.ultima_lectura

    def establecer_parametros(self, **parametros_de_caminata) -> None:
        """
        Establece los parámetros de caminata del robot. Los argumentos pueden ser
        cualquiera de los parámetros de caminata, como 'init_x_offset', 'init_y_offset',
        'init_z_offset', etc. Cambiar estos parámetros afecta directamente la forma en
        que el robot camina, como la amplitud de los pasos o la inclinación del cuerpo.
        """
        parametros = {  # VALORES PREDETERMINADOS
            'init_x_offset': -0.010,  # Ajuste lateral inicial (derecha/izquierda)
            'init_y_offset': 0.005,  # Ajuste longitudinal inicial (adelante/atrás)
            'init_z_offset': 0.020,  # Altura inicial del robot
            'init_roll_offset': np.deg2rad(0.0),  # Inclinación lateral inicial. Controla tobillos y poco de cadera
            'init_pitch_offset': np.deg2rad(-3.0),  # Inclinación frontal inicial. Controla tobillos y poco de cadera
            'init_yaw_offset': np.deg2rad(0.0),  # Rotación vertical inicial. Controla tobillos y poco de cadera
            'period_time': 0.600,  # Duración de cada paso (más rápido/más lento)
            'dsp_ratio': 0.10,  # Tiempo de doble soporte (estabilidad)
            'step_fb_ratio': 0.28,  # Tamaño del paso adelante/atrás
            'x_move_amplitude': 0.00,  # Amplitud de movimiento lateral
            'y_move_amplitude': 0.00,  # Amplitud de movimiento adelante/atrás
            'z_move_amplitude': 0.02,  # Altura del levantamiento del pie
            'angle_move_amplitude': np.deg2rad(0.0),  # Rotación por paso (Yaw Move Amplitude)
            'move_aim_on': False,  # Seguimiento de objetivo activado/desactivado
            'balance_enable': True,  # Control de balance activado/desactivado
            'balance_hip_roll_gain': 0.50,  # Ajuste de balance en cadera (lateral)
            'balance_knee_gain': 0.30,  # Ajuste de balance en rodilla
            'balance_ankle_roll_gain': 1.0,  # Ajuste de balance en tobillo (lateral)
            'balance_ankle_pitch_gain': 0.90,  # Ajuste de balance en tobillo (frontal)
            'y_swap_amplitude': 0.020,  # Intercambio en Y (balanceo lateral)
            'z_swap_amplitude': 0.005,  # Intercambio en Z (altura de balanceo)
            'arm_swing_gain': 1.5,  # Ganancia del movimiento de brazos
            'pelvis_offset': np.deg2rad(5),  # Desplazamiento de la pelvis
            'hip_pitch_offset': np.deg2rad(12.6),  # Ajuste de la cadera (inclinación)
            'p_gain': 32,  # Ganancia proporcional (posición)
            'i_gain': 0,  # Ganancia integral (posición)
            'd_gain': 0  # Ganancia derivativa (posición)
        }

        # Actualizar solo los parámetros reconocidos
        parametros.update({k: v for k, v in parametros_de_caminata.items() if k in parametros})

        # Opcional: Advertir sobre parámetros no reconocidos
        claves_no_reconocidas = set(parametros_de_caminata) - set(parametros)

        if claves_no_reconocidas:
            print(
                f"Advertencia: Los siguientes parámetros no son reconocidos"
                f" y no se actualizarán: {claves_no_reconocidas}")

        # Publica los parámetros actualizados
        self.walk_param_set.publish(*parametros.values())

    def pose_aleatoria(self, rango_roll=(-30, 30), rango_pitch=(-10, 10)):
        roll_aleatorio = np.random.uniform(*rango_roll)
        pitch_aleatorio = np.random.uniform(*rango_pitch)

        # Imprimir los valores aleatorios seleccionados
        # print(
        #     f"\nInclinación seleccionada\n\n"
        #     f"Roll: {roll_aleatorio} grados.\n"
        #     f"Pitch: {pitch_aleatorio} grados.")

        self.establecer_parametros(init_roll_offset=np.deg2rad(roll_aleatorio),
                                   init_pitch_offset=np.deg2rad(pitch_aleatorio))

    def obtener_estado(self) -> Optional[Tuple[float, float, float]]:
        """
        Obtiene el estado inicial del robot basado en las últimas lecturas del IMU.

        Esta función espera hasta que se reciba una lectura válida del IMU y luego
        la retorna como el estado inicial. Si no se dispone de una lectura inmediata,
        realiza intentos periódicos hasta obtener una.

        :return: Tuple[float, float, float]
            Retorna una tupla que contiene las lecturas de aceleración en los ejes x, y, z
            del IMU. Cada elemento de la tupla es un valor flotante que representa la
            aceleración en ese eje.
        """
        tiempo_ms_imu = 5  # Ejemplo: 10 ms
        tasa_lectura_imu = rospy.Rate(1000 / tiempo_ms_imu)

        ultima_lectura = self.obtener_ultima_lectura()

        while ultima_lectura is None:
            tasa_lectura_imu.sleep()  # Controla la frecuencia de intentos de lectura
            ultima_lectura = self.obtener_ultima_lectura()
        return tuple(ultima_lectura)

    def realizar_accion(self, accion: int, pitch_ranges: np.ndarray, roll_ranges: np.ndarray):
        """
        Realiza la acción especificada en el robot, ajustando los parámetros de pitch y roll.

        :param accion: int
            El índice de la acción a realizar.
        :param pitch_ranges: np.ndarray
            Un arreglo de Numpy con los rangos de pitch disponibles.
        :param roll_ranges: np.ndarray
            Un arreglo de Numpy con los rangos de roll disponibles.

        Nota: La función asume que el número total de acciones es el producto de la longitud
        de los rangos de pitch y roll. La acción se mapea a una combinación específica de
        pitch y roll basándose en estos rangos.
        """
        num_acciones_roll = len(roll_ranges)
        indice_pitch = accion // num_acciones_roll
        indice_roll = accion % num_acciones_roll

        # En grados porque ya el publish lo tengo para que lo convierta a rad
        valor_pitch = pitch_ranges[indice_pitch]
        valor_roll = roll_ranges[indice_roll]

        # Ajustar los parámetros de pitch y roll del robot
        self.establecer_parametros(init_pitch_offset=np.deg2rad(valor_pitch),
                                   init_roll_offset=np.deg2rad(valor_roll))


# Parámetros de la Q-Tabla y del agente
num_estados = len(rangos_aceleracion_x) * len(rangos_aceleracion_y) * len(rangos_aceleracion_z)
num_acciones = len(rangos_pitch) * len(rangos_roll)
alpha = 0.1  # Tasa de aprendizaje
gamma = 0.9  # Factor de descuento
epsilon = 0.5  # Probabilidad de exploración 0.1 = 10% de probabilidad de acción al azar y 90% de explotar conocimiento

# Crear el agente
agente_rl = AgenteRL(num_estados, num_acciones, alpha, gamma, epsilon)

archivo_estado = '/home/darwin/Documents/DARwInOP2_ws/src/EntrenamientoRL/scripts/estado_entrenamiento.pkl'
historial_recompensas = []

if os.path.exists(archivo_estado):
    print("\n\n¡ARCHIVO DE RESPALDO ENCONTRADO! CARGANDO...\n\n")
    estado_cargado = cargar_estado(archivo_estado)
    agente_rl.q_tabla = estado_cargado['q_tabla']
    # agente_rl.epsilon = estado_cargado.get('epsilon', agente_rl.epsilon)
    agente_rl.alpha = estado_cargado.get('alpha', agente_rl.alpha)
    agente_rl.gamma = estado_cargado.get('gamma', agente_rl.gamma)
    episodio_inicio = estado_cargado['episodio_actual']
    historial_recompensas = estado_cargado.get('historial_recompensas', [])
else:
    print("\n\n¡NO SE ENCONTRÓ NINGÚN ARCHIVO DE RESPALDO! LO SIENTO, INICIANDO UNO DESDE CERO...\n\n")
    episodio_inicio = 0

rospy.sleep(1)  # Espera para asegurar que ROS se haya iniciado correctamente
control_Stella = ControlRobot()
rospy.sleep(2)
control_Stella.publicar('pose_inicial')
rospy.sleep(5)

num_episodios = 100000
duracion_episodio_segundos = 10  # Duración máxima de cada episodio en segundos
tiempo_ms = 1  # Intervalo de tiempo para cada paso en milisegundos
tasa_acciones = rospy.Rate(1000 / tiempo_ms)  # Nueva tasa de acciones

# Calcula cuántos pasos hay en la duración deseada del episodio
pasos_por_segundo = 1000 / tiempo_ms
max_pasos_por_episodio = int(duracion_episodio_segundos * pasos_por_segundo)
final_del_episodio = None

signal.signal(signal.SIGINT, manejar_interrupcion)


for episodio in range(episodio_inicio, num_episodios):
    # rospy.sleep(2)
    # control_Stella.publicar('pose_inicial')
    # rospy.sleep(5)
    control_Stella.publicar('pose_caminata')
    rospy.sleep(2.5)

    control_Stella.pose_aleatoria()
    rospy.sleep(1.0)
    estado = control_Stella.obtener_estado()
    total_recompensas = 0
    finaliza_episodio = False
    motivo_finalizacion = ""

    for paso in range(max_pasos_por_episodio):
        # Convertir el estado actual a un índice de estado
        indice_estado = mapear_estado_a_indice(*estado)

        # Seleccionar el índice de una acción basada en el índice de estado
        indice_accion_seleccionada = agente_rl.seleccionar_accion(indice_estado)

        # Realizar la acción correspondiente al índice seleccionado
        control_Stella.realizar_accion(indice_accion_seleccionada, rangos_pitch, rangos_roll)

        # Comprobar si se ha alcanzado el final del episodio
        finaliza_episodio, motivo_finalizacion = agente_rl.es_final_del_episodio(estado,
                                                                                 rangos_aceleracion_x,
                                                                                 rangos_aceleracion_y,
                                                                                 rangos_aceleracion_z)

        # Obtener el nuevo estado después de realizar la acción
        nuevo_estado = control_Stella.obtener_estado()

        # Convertir el nuevo estado a un índice de estado
        indice_nuevo_estado = mapear_estado_a_indice(*nuevo_estado)

        # Calcular la recompensa basada en el nuevo estado y las condiciones de finalización
        recompensa_paso = agente_rl.calcular_recompensa(nuevo_estado, finaliza_episodio, motivo_finalizacion)

        # Actualizar la tabla Q usando los índices de estado
        agente_rl.actualizar_q_tabla(indice_estado, indice_accion_seleccionada, recompensa_paso, indice_nuevo_estado)

        # Acumular las recompensas totales
        total_recompensas += recompensa_paso

        # Actualizar el estado actual al nuevo estado
        estado = nuevo_estado

        # Esperar el tiempo definido antes de realizar la siguiente acción
        tasa_acciones.sleep()

        if finaliza_episodio:
            print(f"\nEpisodio {episodio + 1} interrumpido en paso: {paso}."
                  f"\nRazón: {motivo_finalizacion}."
                  f"\nRecompensa Final:{total_recompensas}")
            break

    # Guardar estado y manejar el historial de recompensas
    historial_recompensas.append(total_recompensas)
    guardar_estado(agente_rl, episodio, historial_recompensas)
    if finaliza_episodio:
        print("Guardando Estado Interrumpido\n")
    else:
        print(f"\nEpisodio {episodio + 1} completado.\nRecompensa total = {total_recompensas}.\nGuardando Estado.\n")

# TODO.-    Hay que revisar bien los parametros de acciones, creo que no tienen suficientes rangos para actuar, pero
#           igual hay que dejarlo entrenar más tiempo, con 100 iteraciones si empezaba a haber resultados, y hay
#           que empezar a grabra estas cosas :D
