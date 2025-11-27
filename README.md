> ⚠️ **Nota**: Los modelos de visión y checkpoints de Reinforcement Learning no están en este repo por su tamaño.
> Deben estar en Stella, especificamente en:
>
> /home/darwin/Documents/DARwInOP2_ws/src/SLAM/scripts/
> /home/darwin/Documents/DARwInOP2_ws/src/EntrenamientoRL/scripts/estado_entrenamiento.pkl
>
> 
> Contacta a Miguel Rangel o a la Dra. Yesenia Eleonor en que caso de que no esten :v
> 
> Una vez copiado el repositorio dentro de tu carpeta que será el workspace se debe ejecutar
>
> catkin_make # Para compilar
> 
>
> Ejemplo:
> 
> sudo bash # Entas como root
> source devel/setup.bash # Ejecutar programas compilados
> 
> sudo chmod 666 /dev/ttyUSB0 # Se abre el puerto
> 
> roslaunch op2_manager op2_manager.launch # Se ejecuta el manager para usar los motores
