from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import pandas as pd
import rospy


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

        # Subscripción al tópico del IMU
        self.imu_subscriber = rospy.Subscriber("/robotis/cm_740/imu", Imu, self.imu_callback)
        self.ultima_aceleracion_lineal = None
        self.guardar_datos_imu = False
        self.datos_aceleracion_lista = []

    def imu_callback(self, data):
        if self.guardar_datos_imu:
            self.datos_aceleracion_lista.append([
                data.linear_acceleration.x,
                data.linear_acceleration.y,
                data.linear_acceleration.z
            ])

    def guardar_datos_imu_en_csv(self, nombre_archivo):
        df = pd.DataFrame(self.datos_aceleracion_lista, columns=['x', 'y', 'z'])
        df.to_csv(nombre_archivo, index=False)
        print(f"Datos guardados en {nombre_archivo}")

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

    def pose_aleatoria(self, rango_roll, rango_pitch):
        roll_aleatorio = np.random.uniform(*rango_roll)
        pitch_aleatorio = np.random.uniform(*rango_pitch)

        # Imprimir los valores aleatorios seleccionados
        print(
            f"\nInclinación seleccionada\n\n"
            f"Roll: {roll_aleatorio} grados.\n"
            f"Pitch: {pitch_aleatorio} grados")

        self.establecer_parametros(init_roll_offset=np.deg2rad(roll_aleatorio),
                                   init_pitch_offset=np.deg2rad(pitch_aleatorio))


# Definir los rangos para roll y pitch
rango_roll_global = (-20, 20)
rango_pitch_global = (-20, 20)

# Inicializar el nodo ROS y la clase ControlRobot
rospy.sleep(1)
robot = ControlRobot()

# Publicar la pose inicial del robot
rospy.sleep(3)
robot.publicar('pose_inicial')
rospy.sleep(5)
robot.publicar('pose_caminata')
rospy.sleep(3)

input('Listo, púchele padrinote')
# Llevar al robot a una pose aleatoria
robot.pose_aleatoria(rango_roll_global, rango_pitch_global)
robot.guardar_datos_imu = True

# Esperar a que el usuario presione Enter para detener la impresión de datos del IMU
input("Presiona Enter para detener la impresión de datos del IMU...\n")
robot.guardar_datos_imu = False

# Guardar los datos en un archivo CSV
robot.guardar_datos_imu_en_csv("/home/darwin/Documents/DARwInOP2_ws/src/EntrenamientoRL/scripts/datos_aceleracion.csv")

# Aquí puedes incluir cualquier otro código necesario para tu aplicación
# Por ejemplo, iniciar un episodio de RL, etc.
