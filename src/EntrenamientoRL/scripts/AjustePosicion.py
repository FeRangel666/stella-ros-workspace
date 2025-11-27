"""
Este es el primer paso, poder controlar parametros de caminata mientras está caminando,
por el momento soy capaz de inclinar el cuerpo hacia adelante y hacia atrás, siguiente paso
poder controlar la inclinación lateral y frontal.
"""

import rospy
from std_msgs.msg import String
from op3_walking_module_msgs.msg import WalkingParam
import threading
import numpy as np


class ControlRobot:
    """
    Clase para controlar la caminata de un robot usando ROS y Dynamixel SDK.

    Esta clase permite publicar diferentes tipos de poses, iniciar o detener la caminata,
    ajustar los parámetros de la caminata y modificar de manera continua algún parámetro
    durante la caminata.
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

    def publicar(self, tipo_pose: str) -> None:
        """
        Publica la pose especificada por el tipo.

        :param tipo_pose: str
            Tipo de pose a publicar. Puede ser 'pose_inicial' para una pose inicial
            o 'pose_caminata' para activar el módulo de caminata.
        """
        if tipo_pose == 'pose_inicial':
            self.ini_pose.publish('ini_pose')
        elif tipo_pose == 'pose_caminata':
            self.module.publish('walking_module')
        else:
            rospy.loginfo("Tipo de pose no reconocido")

    def estado_caminata(self, iniciar: bool) -> None:
        """
         Inicia o detiene la caminata del robot.

         :param iniciar: bool
             Si es True, inicia la caminata; si es False, la detiene.
         """
        self.caminata_activa = iniciar
        self.walk.publish('start' if iniciar else 'stop')

    def establecer_parametros(self, **parametros_de_caminata) -> None:
        """
        Establece los parámetros de caminata del robot. Los argumentos pueden ser
        cualquiera de los parámetros de caminata, como 'init_x_offset', 'init_y_offset',
        'init_z_offset', etc. Cambiar estos parámetros afecta directamente la forma en
        que el robot camina, como la amplitud de los pasos o la inclinación del cuerpo.
        """

        # print("Valores de parámetros antes de la actualización:", parametros_de_caminata)

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
            'angle_move_amplitude': np.deg2rad(0.0),  # Rotación por paso
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

    def ajustar_continuamente(self, valor_inicial, valor_final):
        """
        Ajusta de manera suave y continua un parámetro entre dos valores a lo largo de una
        duración total especificada en milisegundos.

        :param valor_inicial: float
            Valor inicial del parámetro a ajustar.
        :param valor_final: float
            Valor final hacia el que se quiere mover el parámetro.

        Ideal para crear movimientos dinámicos y fluidos, como ajustar la inclinación
        mientras el robot está caminando.
        """
        duracion_total_ms = 2500  # Duración total de la transición en milisegundos
        rapidez_transicion = 0.05  # Rapidez de la transición, valores más altos para transiciones más rápidas

        duracion_total_s = duracion_total_ms / 1000.0  # Convertir a segundos
        numero_de_pasos = int(duracion_total_s / rapidez_transicion)
        valores = np.linspace(valor_inicial, valor_final, numero_de_pasos)

        rate = rospy.Rate(1.0 / rapidez_transicion)  # Frecuencia para cada paso

        while self.caminata_activa:
            # Itera sobre el arreglo para ir del valor inicial al final
            for valor in valores:
                if not self.caminata_activa:  # Verificar si la caminata sigue activa
                    return
                self.establecer_parametros(init_pitch_offset=valor)
                rate.sleep()

            # Invertir el arreglo para la transición de regreso
            for valor in reversed(valores):
                if not self.caminata_activa:  # Verificar si la caminata sigue activa
                    return
                self.establecer_parametros(init_pitch_offset=valor)
                rate.sleep()


# Espera inicial de 1 segundo para asegurar que todo está listo
rospy.sleep(1)

# Crear una instancia de la clase ControlRobot
robot = ControlRobot()

# Publicar la pose inicial del robot
robot.publicar('pose_inicial')

# Esperar 5 segundos para que el robot adopte la pose inicial
rospy.sleep(5)

# Iniciar el módulo de caminata del robot
robot.publicar('pose_caminata')

# Esperar 3 segundos para que el módulo de caminata esté completamente activo
rospy.sleep(3)

# Solicitar entrada del usuario para continuar, esto es útil para pausar y observar el estado actual
input('Puchele :v ')

# Activar el estado de caminata del robot (comienza a caminar)
robot.estado_caminata(iniciar=True)
# inicio_caminata = rospy.get_time()


# Ajustar roll_offset continuamente en un hilo separado
# Un "hilo" en programación es una secuencia de ejecución independiente que puede correr en paralelo
# al hilo principal de ejecución. Esto permite realizar múltiples tareas al mismo tiempo.

# ajustar_continuamente es el algoritmo para ajustar la inclinación hacia adelante y hacia atrás
# del bobot, a su vez, este método invoca a establecer_parametros que es quien realmente se encarga
# de ajustar la posición del bobot
hilo_ajuste_roll = threading.Thread(target=robot.ajustar_continuamente, args=(np.deg2rad(0), np.deg2rad(30)))

# Iniciar el hilo para ajustar el parámetro 'hip_pitch_offset' de manera continua.
# Esta acción se ejecuta en paralelo al flujo principal del programa, permitiendo que
# el robot siga caminando mientras ajusta su parámetro 'hip_pitch_offset'.
hilo_ajuste_roll.start()

# Detener la caminata después de 10 segundos
rospy.sleep(100)  # Duración de la caminata
robot.estado_caminata(iniciar=False)
# fin_caminata = rospy.get_time()

# Esperar a que el hilo de ajuste termine
# El método 'join()' es usado para esperar a que el hilo termine su ejecución.
# Esto es importante para asegurar que el ajuste de parámetros se complete antes de que
# el programa principal continúe o finalice.
hilo_ajuste_roll.join()
# duracion_caminata = fin_caminata - inicio_caminata
# print(f"Duración de la caminata: {duracion_caminata} segundos")
