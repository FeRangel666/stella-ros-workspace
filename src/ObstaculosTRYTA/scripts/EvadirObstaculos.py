from op3_walking_module_msgs.msg import WalkingParam
from std_msgs.msg import String
from dynamixel_sdk import *
import rospy


def rutina():
    # Giro
    establecer_parametros(x=0.005, a=0.500)  # 0.5
    time.sleep(0.1)
    iniciar_caminata()
    time.sleep(1.2)  # 1.2
    detener_caminata()
    # Caminata
    establecer_parametros(a=-0.009)
    iniciar_caminata()
    time.sleep(6)
    detener_caminata()
    time.sleep(0.6)
    # giro

    # caminata
    establecer_parametros(a=-0.030)
    iniciar_caminata()
    time.sleep(11.4)
    detener_caminata()
    time.sleep(1.2)

    # Giro
    establecer_parametros(x=0.005, a=0.5)
    iniciar_caminata()
    time.sleep(1.2)
    detener_caminata()
    time.sleep(0.1)
    # Caminata
    establecer_parametros(a=0.15)
    iniciar_caminata()


def pose_inical():
    ini_pose.publish('ini_pose')


def pose_intrinseca():
    module.publish('walking_module')


def iniciar_caminata():
    walk.publish('start')


def detener_caminata():
    walk.publish('stop')


def establecer_parametros(x=0.015, y=0.0, z=0.03, a=-0.00, tiempo=0.600):
    # Default setting walking init pose
    init_x_offset = -0.010
    init_y_offset = 0.005
    init_z_offset = 0.020
    init_roll_offset = 0.0
    init_pitch_offset = 0.0  # 0.0
    init_yaw_offset = 0.0
    # time parameter
    period_time = tiempo
    dsp_ratio = 0.10
    step_fb_ratio = 0.28
    # walking parameter ########
    x_move_amplitude = x
    y_move_amplitude = y
    z_move_amplitude = z
    angle_move_amplitude = a
    move_aim_on = False
    # balance parameter ##########
    balance_enable = True
    balance_hip_roll_gain = 0.50
    balance_knee_gain = 0.30
    balance_ankle_roll_gain = 1.0
    balance_ankle_pitch_gain = 0.90
    y_swap_amplitude = 0.020
    z_swap_amplitude = 0.005
    arm_swing_gain = 1.5
    pelvis_offset = 0.0524
    hip_pitch_offset = 0.3  # 0.22
    # gain parameter ##########
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


# if __name__ == '__main__':
rospy.init_node('Nodo publisher shido')
rospy.loginfo("[PUBLISHER  PY  NODE] " + rospy.get_namespace())

module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
walk = rospy.Publisher("/robotis/walking/command", String, queue_size=10)
walk_param_set = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=10)
ini_pose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)

time.sleep(3)
pose_inical()
time.sleep(7)

pose_intrinseca()
time.sleep(5)

input('Listo padrinote, cuando guste!')
rutina()
