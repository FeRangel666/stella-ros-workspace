import math
from matplotlib import pyplot as plt
import numpy as np

show_animation = True


def calc_heuristic(n1, n2):
    w = 1  # weight of heuristic
    d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
    return d


def get_motion_model():
    motion = np.array([[1, 0, 1],
                       [0, 1, 1],
                       [-1, 0, 1],
                       [0, -1, 1],
                       [-1, -1, math.sqrt(2)],
                       [-1, 1, math.sqrt(2)],
                       [1, -1, math.sqrt(2)],
                       [1, 1, math.sqrt(2)]])

    return motion


class Ruta:
    def __init__(self, ox, oy, reso, rr):  # Initialize grid map for a star planning
        """ox: x position list of Obstacles [m], oy: y position list of Obstacles [m]
        reso: grid resolution [m], rr: robot radius[m]
        """
        self.obmap = None

        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxy = None

        self.xwidth = None
        self.ywidth = None

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, robot_x, robot_y, gx, gy):  # A star path search
        """input: sx: start x position [m], sy: start y position [m], gx: goal x position [m], gy: goal y position [m]
           output: rx: x position list of the final path, ry: y position list of the final path"""
        nstart = self.Node(self.calc_xyindex(robot_x, self.minx), self.calc_xyindex(robot_y, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx), self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            plt.plot(self.calc_grid_position(current.x, self.minx), self.calc_grid_position(current.y, self.miny),
                     "xc")

            if current.x == ngoal.x and current.y == ngoal.y:
                print("\n¡Encontró  ruta!")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            del open_set[c_id]  # Remove the item from the open set

            closed_set[c_id] = current  # Add it to the closed set

            for i, _ in enumerate(self.motion):  # expand_grid search grid based on motion model
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):  # If the node is not safe, do nothing
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node  # This path is the best until now. record it
        rx, ry = self.calc_final_path(ngoal, closed_set)
        return rx, ry

    def calc_final_path(self, ngoal, closedset):  # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return np.array(rx), np.array(ry)

    def calc_grid_position(self, index, minp):  # calc grid position
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # print('Hola, esto te causa problemas?', node.x)
        # print('Hola, esto te causa problemas?', node.y)
        if self.obmap[int(node.x)][int(node.y)]:  # collision check
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = np.round(np.min(ox))
        self.miny = np.round(np.min(oy))
        self.maxx = np.round(np.max(ox))
        self.maxy = np.round(np.max(oy))

        self.xwidth = np.round((self.maxx - self.minx) / self.reso).astype(int)
        self.ywidth = np.round((self.maxy - self.miny) / self.reso).astype(int)

        X, Y = np.meshgrid(np.linspace(self.minx, self.maxx, self.xwidth),
                           np.linspace(self.miny, self.maxy, self.ywidth),
                           indexing='ij')

        D = np.hypot(np.subtract.outer(ox, X), np.subtract.outer(oy, Y))
        self.obmap = np.any(D <= self.rr, axis=0)
