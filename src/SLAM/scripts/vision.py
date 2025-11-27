from numpy import asanyarray, dstack, where
from math import sqrt, pow
import pyrealsense2 as rs
import cv2
# import numpy as np
# import math
# import time


class Vision:
    def __init__(self, clipping_distance_m):
        """
        Inicializa la clase Avision
        :param clipping_distance_m: distancia de recorte en metros
        """
        self.filled_depth = None
        self.hole_filling = None

        self.colorized_depth = None
        self.colorizer = None

        self.bg_removed = None
        self.depth_image_3d = None
        self.color_removed = None
        self.color_image = None
        self.color_frame = None
        self.aligned_depth_frame = None
        self.aligned_frames = None
        self.frames = None

        self.clipping_distance = None
        self.clipping_distance_in_meters = None

        self.width, self.height, self.framerates = 424, 240, 30
        self.ctx = rs.context()
        self.pipeline = rs.pipeline(self.ctx)
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.framerates)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.framerates)
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        # print("Depth Scale is: ", self.depth_scale)

        self.set_clipping_distance_m(clipping_distance_m)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.depth_image = 65.7
        self.color_intrin = 0
        # time.sleep(2.0)

    def set_clipping_distance_m(self, meters):
        """
        Establece la distancia de recorte en metros
        :param meters: distancia de recorte en metros
        """
        self.clipping_distance_in_meters = meters
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale

    def get_image_depth(self):
        """
        Este método obtiene un frame de profundidad y lo alinea con un frame de color utilizando la clase rs.align.
        Además, elimina los fondos de objetos más lejanos de clipping_distance_in_meters.
        Devuelve dos imágenes, una con el fondo y otra sin él.
        """
        self.frames = self.pipeline.wait_for_frames()  # Obtiene el conjunto de frames de color y profundidad
        self.aligned_frames = self.align.process(self.frames)  # Alinea el frame de profundidad al frame de color
        self.aligned_depth_frame = self.aligned_frames.get_depth_frame()  # Obtiene los frames alineados
        self.color_frame = self.aligned_frames.get_color_frame()
        self.color_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # ? DUDA DE QUE ES

        self.depth_image = asanyarray(self.aligned_depth_frame.get_data())
        self.color_image = asanyarray(self.color_frame.get_data())

        # Elimina el fondo - Establece los píxeles más lejanos que la distancia de recorte en el color de fondo, 1 = negro
        self.color_removed = 70
        # La imagen de profundidad es de 1 canal, la imagen de color es de 3 canales
        self.depth_image_3d = dstack((self.depth_image, self.depth_image, self.depth_image))

        self.bg_removed = where((self.depth_image_3d > self.clipping_distance) | (self.depth_image_3d <= 0),
                                   self.color_removed, self.color_image)

        return self.color_image, self.bg_removed

    def get_width_objs(self, x, y, w, h, var_limits_inside):
        """
        Obtiene el ancho de un objeto en unas coordenadas específicas.

        Parameters
        ----------
        x : float
            Coordenada (x) de la esquina superior izquierda del rectángulo que contiene al objeto.
        y : float
            Coordenada (y) de la esquina superior izquierda del rectángulo que contiene al objeto.
        w : float
            Ancho del rectángulo que contiene al objeto.
        h : float
            Altura del rectángulo que contiene al objeto.
        var_limits_inside : float
            Distancia en pixeles dentro del rectángulo que se tomará como punto inicial y final para calcular el ancho.

        Returns
        -------
        float
            Ancho del objeto en metros
        """
        # Obtenemos las coordenadas x, y, z del punto inicial y final del ancho del objeto
        ix1, iy1, iz1 = x + var_limits_inside, y + round(h / 2), (
            self.depth_image[y + round(h / 2), x + var_limits_inside]) / 10

        ix2, iy2, iz2 = x + w - var_limits_inside, y + round(h / 2), (
            self.depth_image[y + round(h / 2), x + w - var_limits_inside]) / 10

        # Utilizamos el método get_distance_points para calcular la distancia entre el punto inicial y final
        result = self.get_distance_points(ix1, iy1, iz1, ix2, iy2, iz2)

        return result

    def get_distance_points(self, ix1, iy1, iz1, ix2, iy2, iz2):
        """
        Este método calcula la distancia entre dos puntos en el espacio tridimensional.
        Utiliza las coordenadas de los puntos en el plano de la imagen y la profundidad asociada a cada uno.
        La función deproject_pixel_to_point de RealSense convierte las coordenadas de pixel a coordenadas en el espacio 3D.

        Parameters
        ----------
        ix1: float
            Coordenada (x) del primer punto en el plano de la imagen
        iy1: float
            Coordenada (y) del primer punto en el plano de la imagen
        iz1: float
            Profundidad asociada al primer punto en el plano de la imagen
        ix2: float
            Coordenada (x) del segundo punto en el plano de la imagen
        iy2: float
            Coordenada (y) del segundo punto en el plano de la imagen
        iz2: float
            Profundidad asociada al segundo punto en el plano de la imagen

        Returns
        -------
        float
            La distancia entre los dos puntos en el espacio tridimensional.
        """
        point1 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [ix1, iy1], iz1)
        point2 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [ix2, iy2], iz2)
        return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2) + pow(point1[2] - point2[2], 2))

    def see_depth(self):
        """
        Muestra la imagen de profundidad en tonos de color.
        """
        self.colorizer = rs.colorizer()  # Crea un objeto colorizer de la librería RealSense4

        # Colorea la imagen de profundidad
        self.colorized_depth = asanyarray(self.colorizer.colorize(self.depth_image).get_data())

        cv2.imshow("see", self.colorized_depth)  # Muestra la imagen coloreada

    def hole_filling_depth(self):
        """
        Aplica un filtro de relleno de huecos en el frame de profundidad alineado y muestra el resultado en una ventana de imagen.
        """

        # Crea un objeto colorizer para colorear la imagen de profundidad
        self.colorizer = rs.colorizer()

        # Crea un objeto hole_filling_filter para aplicar el filtro de relleno de huecos
        self.hole_filling = rs.hole_filling_filter()

        # Aplica el filtro de relleno de huecos en el frame de profundidad alineado
        self.filled_depth = self.hole_filling.process(self.aligned_depth_frame)

        # Colorea la imagen de profundidad con el objeto colorizer
        self.colorized_depth = asanyarray(self.colorizer.colorize(self.filled_depth).get_data())

        # Muestra la imagen coloreada en una ventana de imagen con nombre "hole"
        cv2.imshow("hole", self.colorized_depth)
