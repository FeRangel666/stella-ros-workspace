import os
import subprocess

import pandas as pd
import rospy
from std_msgs.msg import String
from CM730 import CM730

def Iniciar(ServosID):
    os.system("rosnode  kill /op2_manager")  # os.system("rosnode kill -a")
    rospy.sleep(3)
    cm730 = CM730()
    cm730.connect()
    cm730.dxl_on()
    cm730.check_ID(150, 210)
    cm730.servo_sync_write_position(ServosID, [*MotorData.iloc[:, 1].tolist()], False)
    cm730.disconnect()
    rospy.sleep(1)

    return cm730


def SubirEscalon(ServosID, val=1):
    os.system("rosnode kill /op2_manager")  # os.system("rosnode kill -a")
    rospy.sleep(2)
    cm730 = CM730()

    cm730.connect()
    cm730.dxl_on()
    cm730.check_ID(150, 210)

    if val == 0:
        input("Press intro plox:")

    for n, Phase in MotorData.iloc[:, 1:].items():
        motorsPosition = Phase.iloc[:].tolist()
        cm730.servo_sync_write_position(ServosID, [*motorsPosition], False)
        rospy.sleep(0.03)


def AvanzarPoquito():
    subprocess.Popen("rosrun op2_manager op2_manager", shell=True)

    rospy.init_node('Caminata Corta')
    ctrl_module_pub = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
    walk_pub = rospy.Publisher("/robotis/walking/command", String, queue_size=10)

    rospy.sleep(3)
    ctrl_module_pub.publish('walking_module')
    rospy.sleep(2)
    walk_pub.publish('start')
    rospy.sleep(4.2)
    walk_pub.publish('stop')


MotorData = pd.read_csv('/home/darwin/Documents/DARwInOP2_ws/src/RetoTRYTA/scripts/CicloEscaleras.csv')

Servos_ID = MotorData.iloc[:, 0].tolist()
ErrorPermitido = 5

Iniciar(Servos_ID)
SubirEscalon(Servos_ID, 0)
AvanzarPoquito()

SubirEscalon(Servos_ID)
AvanzarPoquito()

SubirEscalon(Servos_ID)
