import pandas as pd
import numpy as np
import rospy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
import threading

ruta_archivo_imu = '/home/darwin/Documents/DARwInOP2_ws/src/SubirEscaleras/scripts/datos_imu.csv'
ruta_secuencia_ascenso = '/home/darwin/Documents/DARwInOP2_ws/src/SubirEscaleras/scripts/Secuencia_Ascenso.csv'


class RecolectorIMU:
    def __init__(self, ruta_archivo, rate_hz=50):
        self.ruta_archivo = ruta_archivo
        self.inicializar_csv()
        self.datos_imu_actual = {}

        self.rate_hz = rate_hz
        self.hilo_recoleccion = threading.Thread(target=self.recolectar_datos)
        self.terminar_recoleccion = False

    def iniciar_recoleccion(self):
        self.hilo_recoleccion.start()

    def recolectar_datos(self):
        rate = rospy.Rate(self.rate_hz)
        while not self.terminar_recoleccion and not rospy.is_shutdown():
            # Aquí se podría implementar la lógica de procesamiento si es necesaria.
            rate.sleep()

    def finalizar_recoleccion(self):
        self.terminar_recoleccion = True
        self.hilo_recoleccion.join()

    def inicializar_csv(self):
        try:
            pd.read_csv(self.ruta_archivo)
        except FileNotFoundError:
            df = pd.DataFrame(columns=["aceleracion_x", "aceleracion_y", "aceleracion_z"])
            df.to_csv(self.ruta_archivo, index=False)

    def procesar_datos_imu(self, datos, guardar_en_csv=False):
        datos_imu = {
            "aceleracion_x": datos.linear_acceleration.x,
            "aceleracion_y": datos.linear_acceleration.y,
            "aceleracion_z": datos.linear_acceleration.z,
        }

        self.datos_imu_actual = datos_imu

        if guardar_en_csv:
            df = pd.DataFrame([datos_imu])
            df.to_csv(self.ruta_archivo, mode='a', header=False, index=False)


class ControladorMovimiento:
    def __init__(self, ruta_secuencia):
        self.ciclo_escaleras = pd.read_csv(ruta_secuencia)
        self.publicador_art = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=30)

    def subir_escalon(self):
        modo = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
        modo.publish('none')
        tasa = rospy.Rate(42)  # 42 Hz
        estados_articulacion = JointState()
        estados_articulacion.name = [
            "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
            "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll", "r_hip_pitch", "l_hip_pitch",
            "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
            "head_pan", "head_tilt"]

        estados_articulacion.velocity = []
        estados_articulacion.effort = []

        for _, paso in self.ciclo_escaleras.iloc[:, 1:-1].items():
            fila_radianes = -np.pi + (paso.iloc[:] * (np.pi / 2048))
            estados_articulacion.position = fila_radianes
            self.publicador_art.publish(estados_articulacion)
            tasa.sleep()


def iniciar_subida_de_escaleras():
    rospy.init_node('subida_de_escaleras')
    recolector = RecolectorIMU(ruta_archivo_imu)
    controlador = ControladorMovimiento(ruta_secuencia_ascenso)

    # Configuración inicial del robot
    ini_pose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)
    modo = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)

    rospy.sleep(3)
    ini_pose.publish('ini_pose')
    rospy.sleep(5)
    modo.publish('walking_module')
    rospy.sleep(3)

    input("\n\n Listo! Presiona Enter para iniciar")

    # Iniciar recolección de datos en un hilo separado
    threading.Thread(target=lambda: rospy.Subscriber("/robotis/cm_740/imu", Imu, recolector.procesar_datos_imu,
                                                     {'guardar_en_csv': True})).start()

    controlador.subir_escalon()


if __name__ == "__main__":
    iniciar_subida_de_escaleras()
