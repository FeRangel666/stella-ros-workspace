"""A* grid planning"""
# Based on (credits to): https://github.com/AtsushiSakai/PythonRobotics

# import math
from std_msgs.msg import Int32, String
from matplotlib import pyplot as plt
from datetime import datetime
from ruta import Ruta

import numpy as np
import rospy
import time
import spur

# from IPython.display import clear_output
from time import sleep

plt.rcParams['axes.facecolor'] = 'green'
# set obstable positions
show_animation = True


class Mapeo:
    def __init__(self, obj_robot, event_number):
        """
        Inicializa los atributos de la clase.

        Args:
            obj_robot (objeto): Objeto de la clase Robot.
            event_number (int): Número de evento, 0 para Obstacle run y 1 para Penalty kick.
        """

        self.width_field = 240
        self.height_field = 180
        self.grid_size = 10.0

        # Constructor de robot.py
        self.obj_robot = obj_robot

        self.event_number = event_number  # 0 Obstacle run, 1 Penalty kick

        # current date and time
        self.now = datetime.now()
        self.timestamp = str(datetime.timestamp(self.now))

        self.robot_rad = 10.0  # [cm]

        self.ball_radius = 5

        self.go_x = self.obj_robot.robot_x
        self.go_y = self.obj_robot.robot_y + 20

        self.ox = np.array([])  # posición de lineas
        self.oy = np.array([])

        self.oxx = np.array([])  # posición de obstaculos
        self.oyy = np.array([])

        self.obstacle_goalkeeper_x = np.array([])  # posición de portero
        self.obstacle_goalkeeper_y = np.array([])

        self.offset_obstacles = 5

        self.find_vertical_line = 0  # Saber si hay linea vertical
        self.find_ball = 0
        self.find_goalline = 0

        self.rx = np.array([])  # Ruta rx de trazo
        self.ry = np.array([])  # Ruta ry de trazo
        self.RX = np.array([])  # Ruta a mandar a robot original

        self.Instrucciones = []

        self.finish_event_penalty = False  # Saber si pelota ha llegado a meta

        # COMMENT PLOT
        self.AfieldCompleted()  # LLamar una funcion de la clase misma

    def AfieldCompleted(self):
        ox = np.concatenate((np.arange(0, self.width_field), np.repeat(self.width_field, self.height_field),
                             np.arange(0, self.width_field + 1), np.repeat(0, self.height_field + 1)))
        oy = np.concatenate((np.repeat(0.0, self.width_field), np.arange(0, self.height_field),
                             np.repeat(self.height_field, self.width_field + 1), np.arange(0, self.height_field + 1)))
        self.ox, self.oy = ox, oy

    def Aplot_obstacle(self, obs_x, obs_y, obs_radio):

        if self.width_field >= obs_x >= 0 and self.height_field >= obs_y >= 0:
            oxx = np.array([obs_x])
            oyy = np.array([obs_y])

            # Dar espacio , rango de (1:5), 5 cm
            i = np.arange(1, round(obs_radio / 2))
            oxx = np.concatenate((oxx, obs_x + i, obs_x - i))
            oyy = np.concatenate((oyy, np.repeat(obs_y, len(i) * 2)))

            self.ox = np.concatenate((self.ox, oxx))
            self.oy = np.concatenate((self.oy, oyy))

    def Aplot_goal(self, ball_x, ball_y, radio=None):
        if self.width_field >= ball_x >= 0 and self.height_field >= ball_y >= 0:
            self.finish_event_penalty = True if ball_y > 170 else False
            self.go_x = ball_x
            self.go_y = ball_y

            self.find_ball = 1

    def Aplot_line(self, line_x, line_y, radio=None):
        if self.width_field >= line_x >= 0 and self.height_field >= line_y >= 0:
            self.go_x = line_x
            self.go_y = line_y

            self.find_goalline = 1

    def putpoint_nogoal(self, x, y):
        if self.width_field >= x >= 0 and self.height_field >= y >= 0:
            self.go_x = x
            self.go_y = y

            self.find_vertical_line = 1  # Saber que hay una linea identificada

    def Aplot_goalkeeper(self, x, y, radio=None):
        # self.obj_vision.set_clipping_distance_m((cz_real / 100) + 0.50)
        # print("clipping_distance: ", (cz_real / 100) + 0.50)

        if self.width_field >= x >= 0 and self.height_field >= y >= 0:
            obstacle_goalkeeper_x = np.array([x])
            obstacle_goalkeeper_y = np.array([y])

            self.ox = np.concatenate((self.ox, obstacle_goalkeeper_x))
            self.oy = np.concatenate((self.oy, obstacle_goalkeeper_y))

    def clean_all(self):
        self.ox = np.array([])
        self.oy = np.array([])
        self.oxx = np.array([])
        self.oyy = np.array([])
        self.obstacle_goalkeeper_x = np.array([])
        self.obstacle_goalkeeper_y = np.array([])
        # del self.go_x[:]
        # del self.go_y[:]

    def check_destiny(self):
        if self.find_vertical_line == 0 or self.find_ball == 0 or find_goalline == 0:
            self.go_x = self.obj_robot.robot_x
            self.go_y = self.obj_robot.robot_y + 20

    def definir_instrucciones(self, rx, ry):
        diferencias = np.diff(rx)
        instrucciones = []

        for diferencia in diferencias:
            if diferencia == 0:
                instrucciones.append(1)
            elif diferencia < 0:
                instrucciones.extend([3, 1])
            elif diferencia > 0:
                instrucciones.extend([4, 1])

        if self.event_number == 0:
            instrucciones.append(6)
        elif self.event_number == 1:
            instrucciones.append(5)

        self.Instrucciones = instrucciones

        # print('rx: ', rx)
        # print('ry: ', ry)
        print('Las instrucciones son: ', *instrucciones)

    # def Aplot_ball_robot(self):
    #     if self.finish_event_penalty:
    #         print("-- Pelota fuera de zona --")
    #         return 0
    #
    #     if self.width_field >= self.go_x >= 0 and self.height_field >= self.go_y >= 0:
    #         if show_animation:
    #             plt.plot(self.ox, self.oy, "sy" if self.event_number == 0 else "sw")
    #             plt.plot(self.obstacle_goalkeeper_x, self.obstacle_goalkeeper_y, "sk" if self.event_number == 1 else '^')
    #             plt.plot(self.oxx, self.oyy, "sb")
    #             plt.plot(self.obj_robot.robot_x, self.obj_robot.robot_y - 5, "^k")
    #             plt.plot(self.go_x, self.go_y + 10, "or")
    #
    #             plt.grid(True)
    #             obj_ruta = Ruta(self.ox, self.oy, self.grid_size, self.robot_rad)
    #             self.rx, self.ry = obj_ruta.planning(self.obj_robot.robot_x, self.obj_robot.robot_y, self.go_x, self.go_y)
    #
    #             if len(self.rx) <= 1:
    #                 return 404
    #
    #             plt.plot(self.rx, self.ry, "xw")
    #
    #             self.rx = np.flip(self.rx)
    #             self.ry = np.flip(self.ry)
    #
    #             if len(self.rx) <= 3:
    #                 if self.event_number == 0:
    #                     return 6
    #                 print("-- Robot ha llegado a destino --")
    #                 if self.event_number == 1:
    #                     print("-- Pateo de balón--")
    #                     return 6
    #             else:
    #                 print('self.rx: ', self.rx, '\n self.ry:', self.ry)
    #                 plt.grid(True)
    #                 plt.axis("equal")
    #                 rutaActual = "/home/darwin/Documents/DARwInOP2_ws/src/Slam/scripts/evidence_image/actual_field_status.png"
    #                 plt.savefig(rutaActual)
    #                 self.definir_instrucciones(self.rx, self.ry)
    #                 self.obj_robot.move_robot(self.rx[1], self.ry[1])
    #
    #             self.clean_all()
    #             plt.clf()
    #             self.AfieldCompleted()
    #             return 1
    #
    #     else:
    #         print("- 400 code error, Posiciones no correctas -")
    #     return None

    def Aplot_ball_robot(self, aasd=[]):
        pose = rospy.Publisher("/robotis/action/page_num", Int32, queue_size=10)
        module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)

        if self.finish_event_penalty:
            print("-- Pelota fuera de zona --")
            return 0

        # pose = rospy.Publisher("/robotis/action/page_num", Int32, queue_size=10)
        # module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)

        # start and goal position, Star robot (x), [cm]
        if self.width_field >= self.go_x >= 0 and self.height_field >= self.go_y >= 0:
            # print("Positions are: True, to be continued...")
            if show_animation:  # pragma: no cover

                # Se puede prescindir o poner que si no es ninguna ingresar de nuevo
                # if self.event_number == 0 or self.event_number == 1:

                # if self.event_number == 1:
                #     plt.plot(self.ox, self.oy, "sw")  # Lines Penalty kick (obstacles)
                #     plt.plot(self.obstacle_goalkeeper_x, self.obstacle_goalkeeper_y, "sk")  # Goalkeeper
                #     plt.plot(self.oxx, self.oyy, "sb")  # Obstacles
                #
                # elif self.event_number == 0:
                #     plt.plot(self.ox, self.oy, "sy")  # Lines Obstacle Run (obstacles)
                #     plt.plot(self.oxx, self.oyy, "sb")  # Obstacles

                # Se hace así pq sí o sí self.event_number será igual a cero o uno
                colors = {1: ("sw", "sk", "sb"), 0: ("sy", "sb")}

                plt.plot(self.ox, self.oy, colors[self.event_number][0])
                plt.plot(self.obstacle_goalkeeper_x, self.obstacle_goalkeeper_y, colors[self.event_number][1])
                #plt.plot(self.oxx, self.oyy, colors[self.event_number][2])

                # TODO ¿porqué menos 5?
                plt.plot(self.obj_robot.robot_x, self.obj_robot.robot_y - 5, "^k")  # Star robot

                # Grafica del destino del robot
                plt.plot(self.go_x, self.go_y + 10, "or")  # Ball position or point
                plt.grid(True)

                #  ######################################33
                obj_ruta = Ruta(self.ox, self.oy, self.grid_size, self.robot_rad)  # Planner
                self.rx, self.ry = obj_ruta.planning(self.obj_robot.robot_x, self.obj_robot.robot_y, self.go_x,
                                                     self.go_y)  # Planning

                # print(self.rx, len(self.rx))
                #  ######################################69

                if len(self.rx) in [0, 1]:
                    return 404

                # Grafica la ruta
                plt.plot(self.rx, self.ry, "xw")

                self.rx = np.flip(self.rx)
                self.ry = np.flip(self.ry)

                self.RX = self.Instrucciones if self.obj_robot.robot_y <= 32 else self.RX

                # self.RX = np.array([1, 1])
                print(self.RX)

                if len(self.rx) <= 3:
                    if self.event_number == 0:
                        return 6
                    print("-- Robot ha llegado a destino --")

                    if self.event_number == 1:
                        print("-- Pateo de balón --")

                        ruta_original = ""
                        for i in np.append(self.RX[:-4], np.array([5])):
                            ruta_original += str(int(i)) + " "
                        # # Implementación de la patadota
                        module.publish('action_module')  # Modo acción del DARwIn
                        time.sleep(0.5)  # Se espera no más pq sí
                        page = 13 if self.go_x < self.obj_robot.robot_x else 12
                        pose.publish(page)  # Página diezytres del action_page (patada izquierda)
                        time.sleep(1)  # Se espera no más pq sí x2
                        #
                        # # Activación del agente DARwIn (el otro pues)
                        shell = spur.SshShell(hostname="192.168.43.212", username="root", password="123",
                                              missing_host_key=spur.ssh.MissingHostKey.accept)
                        # # result1 = shell.run(
                        # #     ["sh", "-c", "yes | sudo /darwin/Linux/project/tutorial/ball_following/ball_following"])
                        comando = f"yes | sudo /darwin/Linux/project/tutorial/ball_following/ball_following {ruta_original}"
                        # print(comando)
                        shell.run(
                            ["sh", "-c", comando])

                        return 6  # "Continuar"

                else:  # TODO HAY QUE VER SI LLEGA A ESTE ELSE
                    print('Ruta en (x): ', self.rx, '\nRuta en (y):', self.ry)  # Print ruta

                    plt.grid(True)
                    plt.axis("equal")
                    # plt.show(block=False) #Para que no se congele la ejecución
                    plt.savefig(
                        '/home/darwin/Documents/DARwInOP2_ws/src/SLAM/scripts/evidence_image/actual_field_status.png')
                    self.definir_instrucciones(self.rx, self.ry)
                    self.obj_robot.move_robot(self.rx[1], self.ry[1])

                self.clean_all()
                plt.clf()
                self.AfieldCompleted()
                return 1

        else:
            print("- 400 code error, Posiciones no correctas -")
        return None
