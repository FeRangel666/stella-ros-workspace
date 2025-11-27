import numpy as np
import rospy
import time
import os
import sys

sys.path.insert(1, '/home/darwin/Documents/DARwInOP2_ws/src/SLAM/scripts')

from dynamixel_sdk import *
# from subprocess import call
# from CM730 import MX28, CM730
from std_msgs.msg import String, Int32, Header
from op3_walking_module_msgs.msg import WalkingParam
from sensor_msgs.msg import JointState
from main import Main


# from vision import Avision
# from procesamiento import Aprocesamiento
# from op3_online_walking_module_msgs.msg import FootStepCommand


def Pose_Inicial():
    ini_pose.publish('ini_pose')


def Pose_PatadaD():
    pose.publish(12)


def Pose_PatadaI():
    pose.publish(13)


def Pose_Action():
    module.publish('action_module')


def Pose_Caminata():
    module.publish('walking_module')


def Caminata_On():
    walk.publish('start')


def Caminata_Off():
    walk.publish('stop')


# def Giro_Der(index):
# 	turn.publish(*derecha)
#
# def Giro_Izq(index):
# 	turn.publish(*izquierda)

def Pose_none():
    module.publish('none')


def Pose_head():
    module.publish('head_control_module')


def Move_head():
    head.publish(Header(), ["head_pan", "head_tilt"], [0.0, -0.30], [], [])


# def Get_param():
#	walk_param_get.publish(True)

def Set_param(z_move_amplitude=0.03, angle_move_amplitude=0.0):
    # Default setting walking init pose
    init_x_offset = -0.010
    init_y_offset = 0.005
    init_z_offset = 0.020
    init_roll_offset = 0.0
    init_pitch_offset = 0.0
    init_yaw_offset = 0.0
    ########## time parameter
    period_time = 0.600
    dsp_ratio = 0.10
    step_fb_ratio = 0.28
    ########## walking parameter ########
    x_move_amplitude = 0.030
    y_move_amplitude = 0.0
    # z_move_amplitude = 0.030
    # angle_move_amplitude = 0.0
    move_aim_on = False
    ########## balance parameter ##########
    balance_enable = True
    balance_hip_roll_gain = 0.50
    balance_knee_gain = 0.30
    balance_ankle_roll_gain = 1.0
    balance_ankle_pitch_gain = 0.90
    y_swap_amplitude = 0.020
    z_swap_amplitude = 0.005
    arm_swing_gain = 1.5
    pelvis_offset = 0.0524
    hip_pitch_offset = 0.22
    ########## gain parameter ##########
    p_gain = 32
    i_gain = 0
    d_gain = 0

    walk_param_set.publish(
        *[
            init_x_offset, init_y_offset, init_z_offset, init_roll_offset, init_pitch_offset,
            init_yaw_offset, period_time, dsp_ratio, step_fb_ratio, x_move_amplitude, y_move_amplitude,
            z_move_amplitude,
            angle_move_amplitude, move_aim_on, balance_enable, balance_hip_roll_gain, balance_knee_gain,
            balance_ankle_roll_gain, balance_ankle_pitch_gain, y_swap_amplitude, z_swap_amplitude, arm_swing_gain,
            pelvis_offset, hip_pitch_offset, p_gain, i_gain, d_gain
        ]
    )


# ID_HOM_DER = 1
# ID_HOM_IZQ = 2
#
# ID_BRA_DER = 3
# ID_BRA_IZQ = 4
#
# ID_COD_DER = 5
# ID_COD_IZQ = 6
#
# ID_CAD_DER = 7
# ID_CAD_IZQ = 8
#
# ID_GLU_DER = 9
# ID_GLU_IZQ = 10
#
# ID_PNA_DER = 11
# ID_PNA_IZQ = 12
#
# ID_ROD_DER = 13
# ID_ROD_IZQ = 14
#
# ID_TOB_DER = 15
# ID_TOB_IZQ = 16
#
# ID_GAS_DER = 17
# ID_GAS_IZQ = 18
#
# ID_CUE = 19
# ID_CAB = 20
#
# ErrorPermitido = 5  # Dynamixel MX delta
# index = 1
#
# MinMaxHom_D = [180, 180, 180, 180, 180, 180, 180]  # Goal position of Dynamixel MX
# MinMaxHom_I = [2550, 2550, 2550, 2550, 2550, 2550, 2550]  # Goal position of Dynamixel MX
#
# MinMaxBra_D = [1490, 1490, 1490, 1490, 1490, 1490, 1490]  # Goal position of Dynamixel MX
# MinMaxBra_I = [2170, 2170, 2170, 2170, 2170, 2170, 2170]  # Goal position of Dynamixel MX
#
# MinMaxCod_D = [3800, 3800, 3800, 3800, 3800, 3800, 3800]  # Goal position of Dynamixel MX
# MinMaxCod_I = [1710, 1710, 1710, 1710, 1710, 1710, 1710]  # Goal position of Dynamixel MX
#
# MinMaxCad_D = [2080, 2080, 2080, 2080, 2080, 2080, 2080]  # Goal position of Dynamixel MX
# MinMaxCad_I = [2260, 2260, 2260, 2260, 2260, 2260, 2260]  # Goal position of Dynamixel MX
#
# MinMaxGlu_D = [2040, 2040, 2040, 2040, 2040, 2040, 2040]  # Goal position of Dynamixel MX
# MinMaxGlu_I = [2060, 2060, 2060, 2060, 2060, 2060, 2060]  # Goal position of Dynamixel MX
#
# MinMaxPna_D = [1360, 1615, 1615, 1615, 1615, 1615, 1615]  # Goal position of Dynamixel MX
# MinMaxPna_I = [2680, 2400, 2400, 2400, 2600, 2600, 2400]  # Goal position of Dynamixel MX
#
# MinMaxRod_D = [3520, 2690, 2690, 2730, 2730, 2730, 2730]  # Goal position of Dynamixel MX 2690
# MinMaxRod_I = [570, 1430, 1430, 1290, 1290, 1290, 1430]  # Goal position of Dynamixel MX 1430
#
# MinMaxTob_D = [2875, 2400, 2420, 2420, 2420, 2480, 2420]  # Goal position of Dynamixel MX
# MinMaxTob_I = [2215, 2700, 2610, 2460, 2700, 2700, 2700]  # Goal position of Dynamixel MX 2610
#
# MinMaxGas_D = [2020, 2030, 1930, 1945, 1945, 1945, 2100]  # Goal position of Dynamixel MX 1930
# MinMaxGas_I = [3610, 3565, 3535, 3500, 3535, 3535, 3630]  # Goal position of Dynamixel MX
#
# MinMaxCue = [2074, 2074, 2074, 2074, 2074, 2074, 2074]  # Goal position of Dynamixel MX
# MinMaxCab = [2380, 2635, 2635, 2635, 2635, 2635, 2635]  # Goal position of Dynamixel MX
#
# packetHandler = PacketHandler(1.0)


if __name__ == '__main__':
    rospy.init_node('Nodo publisher shido')
    rospy.loginfo("[PUBLISHER  PY  NODE] " + rospy.get_namespace())

    ini_pose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)
    pose = rospy.Publisher("/robotis/action/page_num", Int32, queue_size=10)
    module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
    walk = rospy.Publisher("/robotis/walking/command", String, queue_size=10)
    # walk_param_get = rospy.Publisher("/robotis/walking/get_params", bool, queue_size=10)
    walk_param_set = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=10)
    head = rospy.Publisher("/robotis/head_control/set_joint_states", JointState, queue_size=10)

    # turn	= rospy.Publisher("/robotis/online_walking/foot_step_command", FootStepCommand, queue_size=10)

    # derecha = ['forward', "", step_r, step_time_r, step_len_r, side_len_r, step_ang_r]
    # izquierda = ['turn left', "", step_l, step_time_l, step_len_l, side_len_l, step_ang_l]
    # cm730 = CM730()
    # cm730.connect()
    # cm730.check_ID(199, 201)
    # time.sleep(0.5)
    # cm730.servo_sync_write_position([ID_CUE, ID_CAB], [1, 1])
    time.sleep(7)

    # Pose_Action()
    # time.sleep(2)
    # pose.publish(1)
    # time.sleep(7)
    # pose.publish(1)
    # time.sleep(5)
    # pose.publish(6)
    # time.sleep(5)
    # pose.publish(29)
    # time.sleep(5)
    # pose.publish(41)
    # time.sleep(5)

    # pose.publish(2)
    # time.sleep(5)
    # pose.publish(3)
    # time.sleep(5)
    # pose.publish(4)
    # time.sleep(5)
    # pose.publish(23)
    # time.sleep(5)
    # pose.publish(24)
    # time.sleep(10)
    # pose.publish(27)
    # time.sleep(5)
    # pose.publish(31)
    # time.sleep(10)
    # pose.publish(38)
    # time.sleep(10)
    # pose.publish(57)
    # time.sleep(5)
    # pose.publish(54)
    # time.sleep(10)

    # pose.publish(17)
    # time.sleep(25)
    # pose.publish(90)
    # time.sleep(10)
    # pose.publish(10)
    # time.sleep(10)
    # pose.publish(91)
    # time.sleep(10)
    # pose.publish(11)
    # time.sleep(10)

    # pose.publish(70)
    # time.sleep(5)
    # pose.publish(71)
    # time.sleep(5)
    # pose.publish(12)
    # time.sleep(5)
    # pose.publish(13)
    # time.sleep(10)
    # pose.publish(237)
    # time.sleep(5)
    # pose.publish(239)
    # time.sleep(12)
    # pose.publish(15)
    # time.sleep(5)

    #
    #
    #
    # Pose_Inicial()
    # time.sleep(10)
    #
    # #time.sleep(7)
    #
    #
    Pose_Caminata()
    time.sleep(5)
    #
    #
    # # Caminata_On()
    # # time.sleep(2)
    # # Caminata_Off()
    # # time.sleep(2)
    # # Pose_Action()
    # # time.sleep(1)
    # # Pose_PatadaD()
    # # #call(["rosnode kill /op2_manager"], shell=True)
    # # time.sleep(3)
    # #
    # # Pose_Caminata()
    # # time.sleep(2)
    # # Set_param(0.03,-0.15)
    # # time.sleep(1)
    # # Caminata_On()
    # # time.sleep(2)
    # # Caminata_Off()
    # # time.sleep(5)
    # # #Pose_Inicial()
    # # #time.sleep(3)
    # # Set_param()
    #
    #
    Pose_head()
    time.sleep(1)
    Move_head()
    time.sleep(1)

    obj_main = Main()
    status = False
    while not status:
        status = obj_main.main()

    # time.sleep(0.5)
    # cm730.servo_sync_write_position([ID_CUE, ID_CAB], [1, 1])
    # time.sleep(1)
    # cm730.servo_sync_write_position([ID_PNA_IZQ, ID_TOB_IZQ, ID_GAS_DER], [0.5, -0.08, 0.08])
    # time.sleep(1)
    # call(["roslaunch op2_manager op2_manager.launch"], shell=True)
    # time.sleep(8)
    # Pose_Caminata()
    rospy.spin()
