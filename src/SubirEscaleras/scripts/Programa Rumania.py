from sensor_msgs.msg import JointState
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import pandas as pd
import numpy as np
import rospy
import time
import csv
import os
import signal

datos_imu_actual = None


def limpieza():
    print("ROS está cerrando. Realizando limpieza...")


def procesar_datos_imu(datos, guardar_en_csv=False):
    global datos_imu_actual

    datos_imu = {
        "orientacion_x": datos.orientation.x,
        "orientacion_y": datos.orientation.y,
        "orientacion_z": datos.orientation.z,
    }

    datos_imu_actual = datos_imu

    if guardar_en_csv:
        with open('/home/darwin/Documents/DARwInOP2_ws/src/RetoTRYTA/scripts/datos_imu.csv', mode='a',
                  newline='') as archivo:
            escritor = csv.DictWriter(archivo, fieldnames=datos_imu.keys())
            escritor.writerow(datos_imu)


def subir_escalon():
    modo.publish('none')
#    tasa = rospy.Rate(42)
    tasa = rospy.Rate(42)
    estados_articulacion = JointState()

    estados_articulacion.name = [
        "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
        "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll", "r_hip_pitch", "l_hip_pitch",
        "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
        "head_pan", "head_tilt"
    ]

    estados_articulacion.velocity = []
    estados_articulacion.effort = []

    idx_cadera_derecha = estados_articulacion.name.index("r_ank_roll")
    idx_cadera_izquierda = estados_articulacion.name.index("l_ank_roll")

    for nombre, paso in ciclo_escaleras.iloc[:, 1:-1].items():
        fila_radianes = -np.pi + (paso.iloc[:] * (np.pi / 2048))

        if datos_imu_actual:
            if datos_imu_actual["orientacion_y"] < 0.00:
                correccion = datos_imu_actual["orientacion_x"] * (4.8 / 100)
                fila_radianes[idx_cadera_izquierda] -= correccion
            elif datos_imu_actual["orientacion_y"] > 0.01:
                correccion = datos_imu_actual["orientacion_x"] * (1 / 100) # 1.2/100
                fila_radianes[idx_cadera_derecha] += correccion

        estados_articulacion.position = fila_radianes.tolist()
        publicador_art.publish(estados_articulacion)
        tasa.sleep()


def avanzar():
    modo.publish('walking_module')
    rospy.sleep(0.1)
    publicador_caminar.publish('start')
    rospy.sleep(5.6)
    publicador_caminar.publish('stop')


ciclo_escaleras = pd.read_csv('/home/darwin/Documents/DARwInOP2_ws/src/RetoTRYTA/scripts/CicloEscaleras.csv')
rospy.init_node('subida_de_escaleras')

publicador_art = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=30)
modo = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
publicador_caminar = rospy.Publisher("/robotis/walking/command", String, queue_size=10)

rospy.on_shutdown(limpieza)

time.sleep(3)
modo.publish('walking_module')
time.sleep(3)

input("\n\n Listo! Presiona Enter para iniciar")

# Descomentar la siguiente línea si se desea guardar datos
# rospy.Subscriber("/robotis/cm_740/imu", Imu, procesar_datos_imu, {'guardar_en_csv': True})
rospy.Subscriber("/robotis/cm_740/imu", Imu, procesar_datos_imu)

subir_escalon()
avanzar()
rospy.sleep(0.8)
subir_escalon()
avanzar()
rospy.sleep(0.8)
subir_escalon()
