import time

import numpy as np
import rospy

from dynamixel_sdk import *
from std_msgs.msg import String, Int32, Header
from op3_walking_module_msgs.msg import WalkingParam


def CajaIzquierda(): #en realidad es derecha
#Giro
    Set_param(x=0.005, a=0.500) #0.5
    time.sleep(0.1)
    Caminata_On()
    time.sleep(1.2) # 1.2
    Caminata_Off()
#Caminata   
    Set_param(a=-0.009) #-0.001deberia ser negativo pero así va un poco a la derecha
    Caminata_On()
    time.sleep(6) #4.8
    Caminata_Off()
    time.sleep(0.6)
#giro
    ##Set_param(x=0.005, a=-0.500)
    #time.sleep(0.1) #
    ##Caminata_On()
    ##time.sleep(0.6) # otro giro , originakl 1.1
    #Caminata_Off()
    #time.sleep(0.1) #
    #Caminata_On()
    #time.sleep(0.6)    
    ##Caminata_Off()
    ##time.sleep(1)
    #Set_param(a=0.11)
#caminata    
    Set_param( a = -0.030) #0.04
    Caminata_On()
    time.sleep(11.4)
    Caminata_Off()
    time.sleep(1.2)

#Giro Ahí se detiene a medio camino
    ##Set_param(x=0.005, a=-0.500)
    ##Caminata_On()
    ##time.sleep(0.6)
    ##Caminata_Off()
    ##time.sleep(0.1) #0.6
#Caminata
    ##Set_param(a=0.001) #-
    ##Caminata_On()
    ##time.sleep(4.2)
    ##Caminata_Off()
    ##time.sleep(1)
#Giro
    Set_param(x=0.005, a=0.5)
    Caminata_On()
    time.sleep(1.2)
    Caminata_Off()
    time.sleep(0.1)
#Caminata
    Set_param(a=0.15) #+
    Caminata_On()
    #time.sleep(4.2)
    #Caminata_Off()




# def CajaDerecha():
#     llegada = True
#
#     pipe = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#     config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
#     profile = pipe.start(config)
#
#     frameset = pipe.wait_for_frames()
#     color_frame = frameset.get_color_frame()
#     color_init = np.asanyarray(color_frame.get_data())
#
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     bottomLeftCornerOfText = (10, 500)
#     fontScale = 1
#     fontColor = (255, 255, 255)
#     lineType = 2
#
#     # # PRIMER CASO (caja a la derecha):
#     # y = 0.030
#     Set_param(x=0.005, a=-0.500)
#     Caminata_On()
#     time.sleep(0.6)
#     Caminata_Off()
#     time.sleep(1.2)
#     Set_param(a=0.00)
#     a = True
#
#     # try:
#     while a:
#
#         # Store next frameset for later processing:
#         frameset = pipe.wait_for_frames()
#         color_frame = frameset.get_color_frame()
#         depth_frame = frameset.get_depth_frame()
#
#         color = np.asanyarray(color_frame.get_data())
#         res = color.copy()
#         hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
#
#         # l_b = np.array([24, 133, 48])
#         # u_b = np.array([39, 200, 181])
#         l_b = np.array([15, 100, 100])
#         u_b = np.array([45, 200, 200])
#         # u_b = np.array([44.8, 51.7, 56.9])
#         # l_b = np.array([45.4, 88.9, 76.7])
#
#         mask = cv2.inRange(hsv, l_b, u_b)
#         color = cv2.bitwise_and(color, color, mask=mask)
#
#         colorizer = rs.colorizer()
#         colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
#
#         # Create alignment primitive with color as its target stream:
#         align = rs.align(rs.stream.color)
#         frameset = align.process(frameset)
#
#         # Update color and depth frames:
#         aligned_depth_frame = frameset.get_depth_frame()
#         colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
#
#         ### motion detector
#         d = cv2.absdiff(color_init, color)
#         gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)
#         _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
#         dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
#         (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         # cv2.drawContours(color, c, -1, (0, 255, 0), 2)
#         color_init = color
#
#         depth = np.asanyarray(aligned_depth_frame.get_data())
#
#         Caminata_On()
#
#         for contour in c:
#             if cv2.contourArea(contour) < 1500:
#                 continue
#             (x, y, w, h) = cv2.boundingRect(contour)
#             bottomLeftCornerOfText = (x, y)
#
#             # Crop depth data:
#             depth = depth[x:x + w, y:y + h].astype(float)
#
#             depth_crop = depth.copy()
#
#             if depth_crop.size == 0:
#                 continue
#             depth_res = depth_crop[depth_crop != 0]
#
#             # Get data scale from the device and convert to meters
#             depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
#             depth_res = depth_res * depth_scale
#
#             if depth_res.size == 0:
#                 continue
#
#             dist = min(depth_res)
#             cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3)
#             text = "Depth: " + str("{0:.2f}").format(dist)
#             cv2.putText(res,
#                         text,
#                         bottomLeftCornerOfText,
#                         font,
#                         fontScale,
#                         fontColor,
#                         lineType)
#
#             # PRIMER CASO (caja a la izquierda):
#             # Set_param(x=0.001, y=0.030)
#             # Caminata_On()
#             # time.sleep(6.0)
#             # Caminata_Off()
#             # time.sleep(3)
#             # Set_param()
#             # Caminata_On()
#             # time.sleep(6.0)
#             # Caminata_Off()
#             # time.sleep(3)
#             # Caminata_On()
#             if 0.30 < dist < 0.40 and llegada:
#                 Caminata_Off()
#                 time.sleep(0.6)
#                 Set_param(x=0.005, a=0.500)
#                 Caminata_On()
#                 time.sleep(1.2)
#                 Caminata_Off()
#                 time.sleep(0.6)
#                 Set_param(a=-0.04)
#                 Caminata_On()
#
#                 # time.sleep(0.5)
#                 # Set_param(-0.030)
#                 # time.sleep(0.5)
#                 # Caminata_On()
#                 # time.sleep(1)
#                 llegada = False
#
#             elif 0.30 < dist < 0.40 and not llegada:
#                 # Caminata_Off()
#                 a = False
#                 break
#                 # llegada = True
#
#         cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('RBG', res)
#         cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('Depth', colorized_depth)
#         cv2.namedWindow('mask', cv2.WINDOW_AUTOSIZE)
#         cv2.imshow('mask', mask)
#
#         cv2.waitKey(1)
#
#     # finally:
#     #     pipe.stop()
#     # rospy.spin()


def Pose_Action():
    module.publish('action_module')


def Ini():
    ini_pose.publish('ini_pose')


def Pose_Caminata():
    module.publish('walking_module')


def Caminata_On():
    walk.publish('start')


def Caminata_Off():
    walk.publish('stop')


def Set_param(x=0.015, y=0.0, z=0.03, a=-0.00, Time=0.600):
    # Default setting walking init pose
    init_x_offset = -0.010
    init_y_offset = 0.005
    init_z_offset = 0.020
    init_roll_offset = 0.0
    init_pitch_offset = 0.0 #0.0
    init_yaw_offset = 0.0
    ########## time parameter
    period_time = Time
    dsp_ratio = 0.10
    step_fb_ratio = 0.28
    ########## walking parameter ########
    x_move_amplitude = x
    y_move_amplitude = y
    z_move_amplitude = z
    angle_move_amplitude = a
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
    hip_pitch_offset = 0.3 #0.22
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


# if __name__ == '__main__':
rospy.init_node('Nodo publisher shido')
rospy.loginfo("[PUBLISHER  PY  NODE] " + rospy.get_namespace())

module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
walk = rospy.Publisher("/robotis/walking/command", String, queue_size=10)
walk_param_set = rospy.Publisher("/robotis/walking/set_params", WalkingParam, queue_size=10)
ini_pose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)

time.sleep(3)
Ini()
time.sleep(7)

# time.sleep(3)

Pose_Caminata()
time.sleep(5)

# Lado = int(input('Elige el lado del que está la primer caja. Derecha (0), Izquierda (1): '))

input('Listo padrinote, cuando guste!')
CajaIzquierda()

# if Lado == 0:
#     CajaIzquierda()
# else:
#     CajaDerecha()
