from procesamiento import Procesamiento
from vision import Vision


class Main:
    """
    Esta clase se encarga de inicializar y controlar el flujo del programa principal.
    """

    def __init__(self):
        """
        Inicializa las variables necesarias para el programa principal.
        """
        self.event = 0  # 0 Penalty kick / 1 Obstacle run
        self.clipping_distance_m = 2.0  # Vista distancia de cámara
        self.robot_x = 0
        self.robot_y = 0

    def set_event(self):

        # event_number = input("\n¡BIENVENIDO!\nIngrese el modo ('q' para salir)\n\t\t0 Para obstacle run || 1 Para penalty kick: ")
        print("\n\n¡BIENVENIDO!\nIngrese el modo ('q' para salir)\n\t\t0 Para obstacle run || 1 Para penalty kick:\n ")
        event_number = "1"

        if event_number.lower() == "q":
            print("\n\n\t¡PROGRAMA TERMINADO!")
            exit()
        while event_number not in ["0", "1"]:
            event_number = input(
                "\n¡ERROR!\n\tIngresa un valor válido (0 ó 1) (q para salir): \n\t\t0 Para obstacle run || 1 Para penalty kick: \n Se ha seleccionado Penalty Kick en automático")
            if event_number.lower() == "q":
                print("\n\n\t¡PROGRAMA TERMINADO!")
                exit()
        self.event = int(event_number)

    def robot_position(self):
        """
        Pide al usuario ingresar la posición del robot en (x) & (y).
        """
        # self.robot_x = int(input("\n\tPosición robot en x: "))
        # self.robot_y = int(input("\tPosición robot en y: "))

        self.robot_x = 70
        self.robot_y = 10

    def main(self):
        """
        Inicia el programa y controla el flujo de ejecución.
        """
        # print(__file__ + " INICIAMOS!!")
        self.set_event()
        self.robot_position()

        # Constructor de vision.py
        obj_vision = Vision(self.clipping_distance_m)

        # Constructor de procesamiento.py
        obj_procesamiento = Procesamiento(obj_vision, int(self.event),
                                          self.robot_x,
                                          self.robot_y)  # Inicializar variables en A_vision -> clase Acamera

        STATUS = obj_procesamiento.main()
        return STATUS


if __name__ == '__main__':
    obj_main = Main()
    while True:
        try:
            status = obj_main.main()
            if status:
                break
        except KeyboardInterrupt:
            print("\n\n\t¡PROGRAMA TERMINADO!")
            break
