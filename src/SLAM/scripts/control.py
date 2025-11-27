# Clase robot
import time
import rospy
import math
import numpy as np
from time import sleep
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32
from op3_walking_module_msgs.msg import WalkingParam



class Control:
    def __init__(self, robot_x, robot_y):
        """
        Constructor de la clase Acontrol.
        Inicializa las variables de posición y ángulo del robot.

        Parameters
        ----------
        robot_x : float
            Posición en el eje x del robot.
        robot_y : float
            Posición en el eje y del robot.
        """
        self.angle_robot_camera = 90 - 30
        self.robot_radius = 10  # Radio del robot.
        self.angle_robot_camera_x = 40
        self.angle_robot_camera_y = 60
        self.angle_y = -0.3
        self.angle_x = 0.0

        self.robot_x = robot_x
        self.robot_y = robot_y

    def set_angle_robot_camera_x(self):
        """
        Establece el ángulo de la cámara del robot en el eje x.
        """
        self.angle_robot_camera_x = 20

    def set_angle_robot_camera_y(self):
        """
        Establece el ángulo de la cámara del robot en el eje y.
        """
        self.angle_robot_camera_y = 45

    def move_robot(self, rx, ry):
        """
        Mueve el robot a una posición específica.

        Parameters
        ----------
        rx : int
            Posición en el eje x a la que se moverá el robot.
        ry : int
            Posición en el eje y a la que se moverá el robot.
        """
        ################################################
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
        z_move_amplitude = 0.03
        angle_move_amplitude = np.deg2rad(-3)
        move_aim_on = True
        ########## balance parameter ##########
        balance_enable = True
        balance_hip_roll_gain = 0.50
        balance_knee_gain = 0.30
        balance_ankle_roll_gain = 1.0
        balance_ankle_pitch_gain = 0.90
        y_swap_amplitude = 0.020
        z_swap_amplitude = 0.005
        arm_swing_gain = 1.5
        pelvis_offset = np.deg2rad(3)
        hip_pitch_offset = np.deg2rad(13)
        ########## gain parameter ##########
        p_gain = 32
        i_gain = 0
        d_gain = 0

        pos_x = self.robot_x
        pos_y = self.robot_y
        x = rx - pos_x
        y = ry - pos_y
        d = math.sqrt((x * x) + (y * y))
        pasos = math.ceil(d / (x_move_amplitude * 100))
        t_pasos = (pasos * period_time) / 2

        if x != 0:
            a = np.deg2rad(math.tan(y / x))  # ((math.sin(y/(x))) * (2*math.pi)) / 360
        else:
            a = 0

        self.robot_x = rx
        self.robot_y = ry  # Establecer posicion
        walk = rospy.Publisher("/robotis/walking/command", String, queue_size=10)
        module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
        walk_param_set = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=10)
        head = rospy.Publisher("/robotis/head_control/set_joint_states", JointState, queue_size=10)
        ################################################


        print('Robot rx: ', rx, '\n ry:', ry)
        self.robot_x = rx
        self.robot_y = ry

        input("Stella se moverá a: " + str(rx) + ", " + str(ry))
        # print("Mover robot a: " + str(rx) + " , " + str(ry))

        #########################################################
        self.angle_y -= 0.04 if self.angle_y > -1.2 else -1.2
        module.publish('head_control_module')
        head.publish(Header(), ["head_pan", "head_tilt"], [self.angle_x, self.angle_y], [], [])
        time.sleep(1)

        module.publish('walking_module')
        if pos_x < rx:
            x_move_amplitude = 0.0
            y_move_amplitude = -0.07
            z_move_amplitude = 0.03
            walk_param_set.publish(
                *[
                    init_x_offset, init_y_offset, init_z_offset, init_roll_offset, init_pitch_offset,
                    init_yaw_offset, period_time, dsp_ratio, step_fb_ratio, x_move_amplitude, y_move_amplitude,
                    z_move_amplitude,
                    angle_move_amplitude, move_aim_on, balance_enable, balance_hip_roll_gain, balance_knee_gain,
                    balance_ankle_roll_gain, balance_ankle_pitch_gain, y_swap_amplitude, z_swap_amplitude,
                    arm_swing_gain,
                    pelvis_offset, hip_pitch_offset, p_gain, i_gain, d_gain
                ]
            )
            walk.publish('start')
            time.sleep(period_time)
            walk.publish('stop')

        elif pos_x > rx:
            x_move_amplitude = 0.0
            y_move_amplitude = 0.07
            z_move_amplitude = 0.03
            walk_param_set.publish(
                *[
                    init_x_offset, init_y_offset, init_z_offset, init_roll_offset, init_pitch_offset,
                    init_yaw_offset, period_time, dsp_ratio, step_fb_ratio, x_move_amplitude, y_move_amplitude,
                    z_move_amplitude,
                    angle_move_amplitude, move_aim_on, balance_enable, balance_hip_roll_gain, balance_knee_gain,
                    balance_ankle_roll_gain, balance_ankle_pitch_gain, y_swap_amplitude, z_swap_amplitude,
                    arm_swing_gain,
                    pelvis_offset, hip_pitch_offset, p_gain, i_gain, d_gain
                ]
            )
            walk.publish('start')
            time.sleep(period_time)
            walk.publish('stop')

            # time.sleep(0.5)
            # x_move_amplitude = 0.0
            # z_move_amplitude = 0.03
            # print(a)
            # angle_move_amplitude = a
            # walk_param_set.publish(
            #     *[
            #         init_x_offset, init_y_offset, init_z_offset, init_roll_offset, init_pitch_offset,
            #         init_yaw_offset, period_time, dsp_ratio, step_fb_ratio, x_move_amplitude, y_move_amplitude,
            #         z_move_amplitude,
            #         angle_move_amplitude, move_aim_on, balance_enable, balance_hip_roll_gain, balance_knee_gain,
            #         balance_ankle_roll_gain, balance_ankle_pitch_gain, y_swap_amplitude, z_swap_amplitude,
            #         arm_swing_gain,
            #         pelvis_offset, hip_pitch_offset, p_gain, i_gain, d_gain
            #     ]
            # )
            # walk.publish('start')
            # time.sleep(period_time * 5)
            # walk.publish('stop')
            # time.sleep(0.5)
            #
            # x_move_amplitude = 0.003
            # z_move_amplitude = 0.03
            # angle_move_amplitude = 0.0
            # walk_param_set.publish(
            #     *[
            #         init_x_offset, init_y_offset, init_z_offset, init_roll_offset, init_pitch_offset,
            #         init_yaw_offset, period_time, dsp_ratio, step_fb_ratio, x_move_amplitude, y_move_amplitude,
            #         z_move_amplitude,
            #         angle_move_amplitude, move_aim_on, balance_enable, balance_hip_roll_gain, balance_knee_gain,
            #         balance_ankle_roll_gain, balance_ankle_pitch_gain, y_swap_amplitude, z_swap_amplitude,
            #         arm_swing_gain,
            #         pelvis_offset, hip_pitch_offset, p_gain, i_gain, d_gain
            #     ]
            # )
            # walk.publish('start')
            # time.sleep(t_pasos)
            # walk.publish('stop')
            # time.sleep(0.5)
            #
            #
            # x_move_amplitude = 0.0
            # z_move_amplitude = 0.03
            # print(a)
            # angle_move_amplitude = - a
            # walk_param_set.publish(
            #     *[
            #         init_x_offset, init_y_offset, init_z_offset, init_roll_offset, init_pitch_offset,
            #         init_yaw_offset, period_time, dsp_ratio, step_fb_ratio, x_move_amplitude, y_move_amplitude,
            #         z_move_amplitude,
            #         angle_move_amplitude, move_aim_on, balance_enable, balance_hip_roll_gain, balance_knee_gain,
            #         balance_ankle_roll_gain, balance_ankle_pitch_gain, y_swap_amplitude, z_swap_amplitude,
            #         arm_swing_gain,
            #         pelvis_offset, hip_pitch_offset, p_gain, i_gain, d_gain
            #     ]
            # )
            # walk.publish('start')
            # time.sleep(period_time * 5)
            # walk.publish('stop')

        x_move_amplitude = 0.005
        y_move_amplitude = 0.00
        z_move_amplitude = 0.03
        # angle_move_amplitude = 0.0
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
        walk.publish('start')
        time.sleep(t_pasos)
        walk.publish('stop')
        time.sleep(0.5)


        #########################################################

    def actions(self):
        """
        Método vacío.
        """
        pass

    def net_cpp_python(self):
        """
        Método vacío.
        """
        pass
