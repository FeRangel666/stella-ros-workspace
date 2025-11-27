import colorsys
import math
import random
import time
from datetime import datetime

import cv2
import numpy as np
import tensorflow as tf

from control import Control  # Clase de robot
from mapeo import Mapeo  # Import class from a_star

# import the necessary packages

dic_colors = {"lower_color_blue": np.array([95, 100, 20], dtype=np.uint8),  # Azul
              "upper_color_blue": np.array([135, 255, 255], dtype=np.uint8),  # Azul
              "lower_color_green": np.array([30, 0, 0], dtype=np.uint8),
              # 35,100,20,255/25,52,72 ideal->25, 52, 20, Colorverde 30,0,0
              "upper_color_green": np.array([90, 255, 255], dtype=np.uint8),  # 102, 255, 255 Color verde 90,255,255
              "lower_color_white": np.array([0, 0, 168], dtype=np.uint8),  # blanco 0, 0, 212
              "upper_color_white": np.array([172, 111, 255], dtype=np.uint8),  # blanco 131, 255, 255
              "lower_color_yellow": np.array([20, 70, 70], dtype=np.uint8),  # amarillo
              "upper_color_yellow": np.array([35, 255, 255], dtype=np.uint8),  # amarillo
              }


def configBox(pred_bbox):
    """
    Esta función toma un diccionario 'pred_bbox' con las coordenadas de las cajas delimitadoras (bbox) predichas
    y devuelve dos arrays: uno con las cajas delimitadoras y otro con las confidencias de las predicciones.

    Parameters:
    pred_bbox (dict): Diccionario con las coordenadas de las cajas delimitadoras (bbox) predichas.

    Returns:
    tuple: Contiene dos arrays:
        boxes (np.ndarray): Array con las coordenadas de las cajas delimitadoras.
        pred_conf (np.ndarray): Array con las confidencias de las predicciones.
    """
    boxes = None
    pred_conf = None

    for key, value in pred_bbox.items():
        if boxes is None:
            boxes = value[:, :, 0:4]
        else:
            boxes = np.concatenate((boxes, value[:, :, 0:4]), axis=0)
        if pred_conf is None:
            pred_conf = value[:, :, 4:]
        else:
            pred_conf = np.concatenate((pred_conf, value[:, :, 4:]), axis=0)
    return boxes, pred_conf


def preprocessToTF(frame):
    """
    Esta función toma una imagen de entrada y la preprocesa para poder ser utilizada en un modelo de TensorFlow.

    :param frame: Imagen a preprocesar con forma (alto, ancho, canales)
    :return: Imagen preprocesada con forma (1, alto, ancho, canales)
    """
    # Redimensiona la imagen a (416, 416)
    image_data = cv2.resize(frame, (416, 416))

    # Normaliza los valores de los pixeles a rango [0, 1]
    image_data = image_data / 255.

    # Añade una dimensión extra para indicar la cantidad de instancias
    image_data = image_data[np.newaxis, ...].astype(np.float32)

    return image_data

    # Funcion que lee el archivo .names


def read_class_names(class_file_name):
    """
    Este método lee un archivo de nombres de clases y los almacena en un diccionario,
    donde la clave es el ID de la clase y el valor es el nombre de la clase.

    :param class_file_name: Nombre del archivo que contiene los nombres de las clases.
    :return: diccionario con los nombres de las clases.
    """
    names = {}
    with open(class_file_name, 'r') as data:
        for ID, name in enumerate(data):
            names[ID] = name.strip('\n')
    return names


def cen_moments(countours_found):
    """
    Encuentra el centroide del mejor contorno y lo marca

    :param countours_found: contornos encontrados
    :return: coordenadas (x) & (y) del centroide
    """
    M = cv2.moments(countours_found)  # Encontrar el centroide del mejor contorno y marcarlo
    if M["m00"] != 0:
        cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
    else:
        cx, cy = 0, 0  # set values as what you need in the situation
    return cx, cy


def filter_color(image, color):
    """
    Filtra el color de la imagen
    :param image: imagen a ser filtrada
    :param color: color a ser filtrado
    :return: imagen con el color filtrado
    """
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # convierte de RGB a HSV
    color_mask = cv2.inRange(hsv_frame, dic_colors.get("lower_color_" + color),
                             dic_colors.get(
                                 "upper_color_" + color))  # obtiene el rango de colores para el color especificado
    color_total = cv2.bitwise_and(image, image,
                                  mask=color_mask)  # aplica la mascara de color para obtener solo el color especificado
    diff_total = cv2.absdiff(image, color_total)  # obtiene la diferencia entre la imagen original y la imagen filtrada
    return diff_total


def find_contours(frame, color):
    """
    Encuentra los contornos de un color específico en una imagen dada.

    Args:
    - frame (numpy array): imagen en la que se buscaran los contornos
    - color (str): nombre del color en formato "tipo_color" (ej: "verde", "azul", etc)

    Returns:
    - cnts (list of numpy arrays): lista de contornos encontrados
    - thresh (numpy array): imagen binaria en la que los pixeles del color específico son blancos y el resto negros
    """
    frame = cv2.GaussianBlur(frame, (5, 5), 0)  # (7,7),2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convertir frame de BRG a HSV
    thresh = cv2.inRange(hsv, dic_colors.get("lower_color_" + color), dic_colors.get(
        "upper_color_" + color))  # Aplicar umbral a img y extraer los pixeles en el rango de colores
    cnts, h = cv2.findContours(thresh, cv2.RETR_LIST,
                               cv2.CHAIN_APPROX_SIMPLE)  # Encontrar los contornos en la imagen extraída
    return cnts, thresh


class Procesamiento:
    def __init__(self, obj_vision, event_number, robot_x, robot_y):
        """
        Inicializa las variables y objetos necesarios para el procesamiento de imágenes.

        Parameters
        ----------
            - obj_vision: obj
                objeto de visión
            - event_number: int
                número de evento
            - robot_x: float
                posición en x del robot
            - robot_y: float
                posición en y del robot
        """
        self.bg_removed_green_blue = None
        self.bg_removed_green = None

        self.x_width_object = None
        self.x_z_object = None

        self.cn_angle = None
        self.cn_number = None

        self.event_number = event_number

        # Fecha y hora actual
        self.now = datetime.now()
        self.timestamp = str(datetime.timestamp(self.now))

        # Constructor del objeto Control
        self.obj_robot = Control(robot_x, robot_y)
        # Constructor del objeto Amapeo
        self.obj_mapeo = Mapeo(self.obj_robot, self.event_number)

        self.obj_vision = obj_vision

        self.color_intrin = 0  # Ancho del objeto
        self.width_obj = 0  # Ancho de un objeto
        self.color_image = 0  # Imagen a color
        self.bg_removed = 0  # Imagen a color limitada a distancia
        self.var_limits_inside_object = 10  # Pixeles dentro de objeto detectado

        # Arreglos para guardar los valores reales y predichos
        self.real_array_roc = np.array([])
        self.pred_array_roc = np.array([])

        # Crear arreglos con valores predeterminados
        self.numbers_array_obj = np.arange(192)
        self.distance_array_obj = np.array([0])
        self.width_array_obj = np.array([0])

        # Cargar modelo guardado y obtener su firma de inferencia
        ruta = "/home/darwin/Documents/DARwInOP2_ws/src/SLAM/scripts/custom-416"
        self.saved_model_loaded = tf.saved_model.load(ruta)
        self.infer = self.saved_model_loaded.signatures['serving_default']

        name = "/home/darwin/Documents/DARwInOP2_ws/src/SLAM/scripts/obj.names"
        self.file_names = name
        self.fps = 30  # Número de fotogramas deseado (máximo 30)

        self.result_images_id = 0  # Ver identificación en una sola imagen

        self.dic_values = {}  # Diccionario para detección de líneas

        # Cálculo de proporción
        self.gp = 77 / (424 - 1)

    def draw_bbox(self, image, bboxes, class_dir, image_final):
        """
        Dibuja las cajas delimitadoras alrededor de los objetos detectados en la imagen.

        Parameters
        ----------
        image : numpy.ndarray
            Imagen original.
        bboxes : list
            Tupla que contiene las cajas delimitadoras de los objetos detectados, su puntuación, la clase y el número de cajas.
        class_dir : str
            Directorio de las clases.
        image_final : numpy.ndarray
            Imagen final con las cajas delimitadoras dibujadas.

        Returns
        -------
        numpy.ndarray
            Imagen final con las cajas delimitadoras dibujadas.
        """

        # Lee nombres de clases
        classes = read_class_names(class_dir)
        num_classes = len(classes)

        # Calcula dimensiones de la imagen
        image_h, image_w, _ = image.shape

        # Genera tuplas de colores
        hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
        colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))

        # Desordena colores generados
        random.seed(0)
        random.shuffle(colors)
        random.seed(None)

        # Extrae información de bboxes
        out_boxes, out_scores, out_classes, num_boxes = bboxes

        # Calcula coordenadas de cada caja
        box_coors = [(y1 * image_h, x1 * image_w, y2 * image_h, x2 * image_w) for (y1, x1, y2, x2), score, cls in
                     zip(out_boxes[0], out_scores[0], out_classes[0]) if score > 0.5 and 0 <= cls < num_classes]

        # Dibuja caja alrededor de objeto y agrega información a la imagen
        for i, (y1, x1, y2, x2) in enumerate(box_coors):
            class_ind = int(out_classes[0][i])
            bbox_color = colors[class_ind]
            bbox_thick = int(0.6 * (image_h + image_w) / 600)
            c1, c2 = (int(x1), int(y1)), (int(x2), int(y2))
            # print(c1, c2, 'Hola esto te esta causando problemas?')
            cx1, cy2 = int(round(x1 + ((x2 - x1) / 2))), int(round(y1 + ((y2 - y1) / 2)))

            # Dibuja caja alrededor de objeto
            cv2.rectangle(image_final, c1, c2, bbox_color, bbox_thick)

            if classes[class_ind] == "balon":  # 2 ball
                obj_case = 2
                print("hello balon, pelota a la distancia de: ", round(self.obj_vision.depth_image[cy2, cx1] / 10, 2))

            elif classes[class_ind] == "portero":  # 3 goalkeeper
                obj_case = 3
                print("hello portero")

            elif classes[class_ind] == "porteria":  # 4 porteria
                obj_case = 4
                print("hello porteria")

            else:
                return image_final

            self.put_obj_in_map(cx1, cy2, round(self.obj_vision.depth_image[cy2, cx1] / 10, 2), image_final, 0, 0,
                                obj_case)

        return image_final

    def get_real_distance_objs(self, cz_blue):
        """
        Obtiene la distancia Z del objeto dado un valor Z, teniendo en cuenta el ángulo de la cámara del robot.
        Análisis detallado en 4.5.1
        :param cz_blue: valor Z del objeto
        :return: distancia Z del objeto en relación al robot
        """
        A_dis_total = cz_blue * math.sin(math.radians(self.obj_robot.angle_robot_camera_y))
        return A_dis_total

    def get_coordenates_map(self, angle, distance_robot_obj, suma):
        """
        Obtiene las coordenadas para mapear un objeto con respecto al robot
        :param angle: ángulo del objeto con respecto al robot
        :param distance_robot_obj: distancia del objeto al robot
        :param suma: booleano para indicar si se suma (1) o se resta (0) la distancia al ángulo
        :return: coordenadas x,y del objeto en el mapa
        """
        xb = math.sin(math.radians(angle)) * distance_robot_obj
        yb = math.cos(math.radians(angle)) * distance_robot_obj

        xb_object, yb_object = 0, 0

        if suma == 1:
            xb_object = self.obj_robot.robot_x + xb  # robot_x posicion robot en mapa
            yb_object = self.obj_robot.robot_y + yb  # robot_y posicion robot en mapa
        elif suma == 0:
            xb_object = self.obj_robot.robot_x - xb  # robot_x posicion robot en mapa
            yb_object = self.obj_robot.robot_y + yb  # robot_y posicion robot en mapa
        return xb_object, yb_object

    def put_obj_in_map(self, cx, cy, cz, image_final, cnt, var_limits_inside, obj_case):
        cz_real = round(self.get_real_distance_objs(cz), 2)
        if obj_case in (1, 2, 3, 5):
            if obj_case == 1:
                x, y, w, h = cv2.boundingRect(cnt)
                self.width_obj = round(self.obj_vision.get_width_objs(x, y, w, h, var_limits_inside), 2)
            cv2.circle(image_final, (cx, cy), 5, (0, 0, 255), -1)

            self.cn_number = (cx - 1) * (self.gp)
            if cx >= round(self.obj_vision.width / 2):
                self.cn_angle = self.cn_number - 38.5
                x_obs, y_obs = self.get_coordenates_map(self.cn_angle, cz_real, 1)
            else:
                self.cn_angle = 38.5 - self.cn_number
                x_obs, y_obs = self.get_coordenates_map(self.cn_angle, cz_real, 0)

            if obj_case == 1:
                self.obj_mapeo.Aplot_obstacle(x_obs, y_obs, self.width_obj)
            elif obj_case == 2:
                self.obj_mapeo.Aplot_goal(x_obs, y_obs)
            elif obj_case == 3:
                self.obj_vision.set_clipping_distance_m((cz_real / 100) + 0.50)
                self.obj_mapeo.Aplot_goalkeeper(x_obs, y_obs)
                print("clipping_distance: ", (cz_real / 100) + 0.50)
            elif obj_case == 5:
                self.obj_mapeo.Aplot_line(x_obs, y_obs)

        return image_final

    def search_blue(self, image_final, color):
        """
        Función para buscar objetos de color azul en una imagen y agregarlos a un mapa.

        Parámetros:
        self (objeto) - Instancia de la clase actual.
        image_final (numpy.ndarray) - Imagen en la que se buscarán objetos.
        color (str) - Color que se desea buscar en la imagen.

        Devuelve:
        numpy.ndarray - Imagen con los objetos encontrados marcados en el mapa.
        """
        self.x_z_object = 0
        self.x_width_object = 0

        # Encuentra contornos en la imagen y un umbral
        cnts, thresh = find_contours(image_final, color)

        # Filtra los contornos por el área y solo mantiene aquellos que tienen un área mayor a 300
        cnts = [cnt for cnt in cnts if cv2.contourArea(cnt) > 300]
        for cnt in cnts:
            # Encuentra los momentos centrales del contorno
            cx, cy = cen_moments(cnt)
            # Calcula la profundidad del objeto a partir de la imagen de profundidad
            depth = round(self.obj_vision.depth_image[cy, cx] / 10, 2)
            # Agrega el objeto encontrado al mapa
            image_final = self.put_obj_in_map(cx, cy, depth, image_final, cnt, self.var_limits_inside_object, 1)

        return image_final

    def search_lines(self, image_line, image_final, color):
        """
        Busca las líneas en la imagen utilizando el contorno de los objetos y el filtro de Canny.

        :param image_line: imagen en la que se van a buscar las líneas
        :type image_line: numpy array
        :param image_final: imagen final con las líneas dibujadas
        :type image_final: numpy array
        :param color: color a buscar en la imagen
        :type color: string
        :return: imagen final con las líneas dibujadas
        :rtype: numpy array
        """

        # Encuentra los contornos en la imagen y los filtra por tamaño
        contours, thresh = find_contours(image_line, color)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 150]

        # Dibuja los contornos en la imagen
        cv2.drawContours(thresh, contours, -1, (0, 255, 0), 3)

        # Aplica un filtro de umbral adaptativo y el algoritmo de Canny para detectar bordes
        bw = cv2.adaptiveThreshold(thresh, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, -2)
        dst = cv2.Canny(bw, 25, 300, apertureSize=3)
        # cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        # Aplica la Transformada de Hough para detectar líneas
        lines = cv2.HoughLines(dst, 1, np.pi / 180, 40, min_theta=0, max_theta=np.pi)

        # Si se encuentran líneas, se agrupan por inclinación y se dibujan en la imagen final
        if lines is not None:
            theta_grades = [round(line[0][1] * (180 / np.pi), 3) for line in lines]
            unique_thetas = list(set(theta_grades))
            unique_thetas.sort(reverse=True)

            seen = set()
            for theta in unique_thetas:
                if theta not in seen:
                    seen.add(theta)
                    self.dic_values[theta] = lines[theta_grades.index(theta)][0][0]
                    image_final = self.see_lines(image_final, theta)

        return image_final

    def see_lines(self, image_final, nextlistx):
        """
        Dibuja líneas en una imagen usando un ángulo y una longitud.

        Args:
            image_final (np.ndarray): La imagen en la que se dibujarán las líneas.
            nextlistx (np.ndarray): El ángulo (en grados) de cada línea a dibujar.

        Returns:
            np.ndarray: La imagen con las líneas dibujadas.

        """
        # Convertir ángulos a radianes
        angles = nextlistx * np.pi / 180

        # Obtener la longitud para cada ángulo
        get_length = np.vectorize(self.dic_values.get)
        lengths = get_length(nextlistx)

        # Calcular las coordenadas x0 e y0 de cada línea
        x0 = np.cos(angles) * lengths
        y0 = np.sin(angles) * lengths

        # Calcular las coordenadas x1 e y1 y x2 e y2 de cada línea
        x1 = x0 + 10000 * np.sin(-angles)
        y1 = y0 + 10000 * np.cos(angles)
        x2 = x0 - 10000 * np.sin(-angles)
        y2 = y0 - 10000 * np.cos(angles)

        # Convertir las coordenadas a enteros
        x1, y1, x2, y2 = map(np.int32, [x1, y1, x2, y2])

        # Crear un array de líneas (cada línea es una fila con las coordenadas x1, y1, x2, y2)
        lines = np.column_stack((x1, y1, x2, y2)).reshape(-1, 1, 4)

        # Dibujar las líneas en la imagen
        cv2.polylines(image_final, [lines.reshape(-1, 1, 2)], isClosed=False, color=(0, 255, 0), thickness=1,
                      lineType=cv2.LINE_AA)

        # Si hay algún ángulo en el rango [80, 100] y el event_number es igual a 0, dibujar un círculo en (120, 150)
        mask = np.logical_and(80 <= nextlistx, nextlistx <= 100)
        if np.any(mask) and self.event_number == 0:
            cv2.circle(image_final, (120, 150), 5, (0, 0, 255), -1)

        # Devolver la imagen con las líneas dibujadas
        return image_final

    def buscar_objetos(self, frame, image_final):
        """
        Busca objetos en una imagen dada

        Arguments:
        frame -- imagen en la que se buscarán los objetos
        image_final -- imagen final con los objetos detectados

        Returns:
        image_final -- imagen con los objetos detectados y marcados
        """
        # Preprocesar la imagen para usar en TensorFlow
        batch_data = tf.constant(preprocessToTF(frame))

        # Realizar la predicción de las cajas delimitadoras
        pred_bbox = self.infer(batch_data)

        # Obtener la configuración de las cajas delimitadoras
        boxes, pred_conf = configBox(pred_bbox)

        # Realizar la supresión no máxima combinada
        boxes, scores, classes, valid_detections = tf.image.combined_non_max_suppression(
            boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
            scores=tf.reshape(pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])),
            max_output_size_per_class=50,
            max_total_size=50,
        )

        # Almacenar los resultados de la supresión no máxima combinada
        pred_bbox = [boxes.numpy(), scores.numpy(), classes.numpy(), valid_detections.numpy()]

        # Dibujar los cuadros de detección en la imagen final
        image_final = self.draw_bbox(frame, pred_bbox, self.file_names, image_final)

        return np.asarray(image_final)

    def search_objs(self, frame, image_final):
        start_time = cv2.getTickCount()
        batch_data = tf.constant(preprocessToTF(frame))
        pred_bbox = self.infer(batch_data)
        boxes, pred_conf = configBox(pred_bbox)

        boxes, scores, classes, valid_detections = tf.image.combined_non_max_suppression(
            boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
            scores=tf.reshape(pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])),
            max_output_size_per_class=50,
            max_total_size=50,
        )
        pred_bbox = [boxes.numpy(), scores.numpy(), classes.numpy(), valid_detections.numpy()]
        image_final = self.draw_bbox(frame, pred_bbox, self.file_names, image_final)

        result = np.array(image_final, copy=False)

        fps = cv2.getTickFrequency() / (cv2.getTickCount() - start_time)
        print("FPS: %.2f" % fps)
        return result

    # Main
    def main(self):
        image_finalx = None  # Inicializamos la variable antes de su uso
        Hori = None
        image_final = None  # Inicializamos la variable antes de su uso

        try:  # Loop de transmisión
            # x_distance = 2.4
            while True:
                # x_distance -= .1
                # Eliminar pixeles mayores a clipping metros
                # self.obj_vision.set_clipping_distance_m(x_distance)
                self.color_image, self.bg_removed = self.obj_vision.get_image_depth()
                self.bg_removed_green = filter_color(self.bg_removed.copy(), "green")  # Filtro color verde
                self.bg_removed_green_blue = filter_color(self.bg_removed_green.copy(),
                                                          "blue")  # Filtro color azul

                if self.event_number == 0 or self.event_number == 1:  # obstacle run and port of Penalty Kick
                    image_final = self.search_blue(self.bg_removed_green.copy(), "blue")  # Search blue obstacles

                    # if self.event_number == 0:
                    #     image_finalx = self.search_lines(self.bg_removed_green_blue.copy(), image_final.copy(),
                    #                                      "yellow")
                    image_finalx = self.search_lines(self.bg_removed_green_blue.copy(), image_final.copy(), "white")

                if self.event_number == 1:  # Penalty kick
                    image_final = self.search_objs(self.bg_removed, image_final)  # solo bg_removed

                # concatanate image Horizontally
                rutaImagen = "/home/darwin/Documents/DARwInOP2_ws/src/SLAM/scripts/evidence_image/actual_field_status.png"
                image_mapeo = cv2.imread(rutaImagen, 1)
                image_mapeo = cv2.resize(image_mapeo, (424, 240))
                if self.event_number == 0:
                    Hori = np.concatenate((self.color_image, image_finalx, image_mapeo),
                                          axis=1)  # Axis 0 vertical, 1 horizontal
                elif self.event_number == 1:
                    Hori = np.concatenate((self.color_image, image_final, image_mapeo),
                                          axis=1)  # Axis 0 vertical, 1 horizontal

                cv2.imshow('Imagen', Hori)

                # cv2.namedWindow("image_final", cv2.WINDOW_AUTOSIZE)
                # cv2.imshow('image_final', image_finalx)

                # COMMENT PLOT
                status_meta = self.obj_mapeo.Aplot_ball_robot()

                print('status_meta ', status_meta)

                if status_meta == 6:
                    return True
                elif status_meta == 1:  # Que debe seguir buscando
                    pass
                elif status_meta == 0:  # Terminar escenario
                    print(" -- Se ha concluido el escenario --")
                    return True
                elif status_meta == 404:
                    pass
                    # return False

                # Salir del bucle si se presiona ESC
                k = cv2.waitKey(5) & 0xFF
                if k == 27:
                    break
                if k == ord("q"):
                    print("That's all folks :)")
                    break

        finally:
            pass
