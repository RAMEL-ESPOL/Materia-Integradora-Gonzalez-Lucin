# Sistema Interactivo de Juego de Posturas para Niños con TDAH

Este proyecto implementa un sistema terapéutico basado en ROS 2, visión por computadora y un robot humanoide simulado. Su propósito es apoyar el tratamiento de niños con TDAH mediante un juego de memoria de posturas guiado por el robot, con registro emocional, retroalimentación auditiva y una aplicación móvil para el terapeuta.

---

## Descripción general

El sistema se compone de tres partes principales:

1. Un conjunto de nodos en ROS 2 encargados de la lógica del juego, validación de posturas y reconocimiento emocional.
2. Un entorno de simulación en Gazebo que representa al robot humanoide.
3. Una aplicación móvil Android conectada por WebSocket para visualizar los resultados del niño y controlar el juego.

---

## Requisitos del sistema

### Sistema operativo recomendado

- Ubuntu 22.04 LTS (Linux)

### ROS 2

Desarrollado para **ROS 2 Humble**. Seguir la [guía oficial de instalación](https://docs.ros.org/en/humble/Installation.html).


## Instalación paso a paso

### 1. Crear el workspace

```bash
mkdir -p ~/ros2tesis_ws/src
cd ~/ros2tesis_ws
```

### 2. Clonar el repositorioEste proyecto implementa un sistema terapéutico basado en ROS 2, visión por computadora y un robot humanoide simulado. Su propósito es apoyar el tratamiento de niños con TDAH mediante un juego de memoria de posturas guiado por el robot, con registro emocional, retroalimentación auditiva y una aplicación móvil para el terapeuta.

```bash
cd src
git clone https://github.com/egonzalezauh/Materia-Integradora-Gonzalez-Lucin.git
cd ..
```

### 3. Instalar dependencias del sistema

```bash
sudo apt install python3-colcon-common-extensions
sudo apt update && sudo apt install -y \
  python3-colcon-common-extensions \
  python3-opencv python3-pip \
  # Cámara, imágenes y TF
  ros-humble-usb-cam \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  # Estado del robot y descripción
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  # Control (ROS 2 Control)
  ros-humble-ros2controlcli \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  # Simulación (usar ros_gz; si usas Gazebo clásico, ver nota abajo)
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  # MoveIt (config y visualización)
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-ros-warehouse \
  ros-humble-moveit-setup-assistant \
  ros-humble-warehouse-ros-mongo \
  # Rosbridge (comunicación con la app)
  python3-autobahn \
  python3-twisted \
  python3-ujson

```

### 4. Instalar dependencias ROS 2 del proyecto

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y



```

### 4.5 Opcional, crear env para tener librerias correctas

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

```

### 5. Compilar el proyecto

```bash
colcon build
source install/setup.bash
```

Agregar al `.bashrc`:

```bash
echo "source ~/ros2tesis_ws/install/setup.bash" >> ~/.bashrc
```

---

## Ejecución del sistema

### 1. Iniciar simulación del robot

```bash
ros2 launch coco_gazebo_sim coco_robot.launch.py
```

Opción alternativa (para computadoras de bajos recursos):

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch coco_gazebo_sim coco_robot.launch.py
```

### 2. Iniciar el servicio principal

```bash
ros2 run coco_services launch_master_service
```

### 3. Conectar la app móvil por WebSocket

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Aplicación móvil

Desarrollada en **Kotlin (Android Studio)**.

1. Abre el proyecto en Android Studio.
2. Conecta un dispositivo físico o emulador.
3. Ejecuta el proyecto.

En la app:

- Ingresa la IP del computador con ROS 2.
- Presiona *Conectar*.
- Inicia el juego o accede a reportes.

---

## Estructura de los paquetes del proyecto

| Paquete | Función |
|--------|---------|
| `posture_game` | Lógica del juego, retroalimentación, estadísticas, validacion de posturas, movimiento del robot, deteccion emocional|
| `coco_gazebo_sim` | Entorno de simulación en Gazebo para visualizar los movimiento del robot|
| `coco_services` | Servicio que permite mandar el nombre del paciente e inicios de launch de forma remota|
| `rosbridge_server` | Comunicación WebSocket con la app móvil |
| `coco_ws` | Contiene toda la informacion del robot, como urdf, sacro, comunicacion con Rviz y gazebo, ademas de implementaciones adicionales |
| `dynamixel_sdk` | Capa de comunicación entre ROS 2 y los servomotores Dynamixel de ROBOTIS|

---

## Estructura del paquete ros2_tesis (logica del juego)

| Archivo | Función |
|--------|---------|
| `game_manage_node.py` | Nodo que controla la lógica del juego, retroalimentación, estadísticas, ayudas|
| `detect_emotions_node.py` | Nodo que procesa imagenes y las cataloga gracias a un modelo entrenado con machine learning |
| `mediapipe_node.py` | Nodo que realiza el procesamiento de imagen con MediaPipe, envia topico de imagenes crudo para ser usado en otro nodo que use Mediapipe|
| `speaker_node.py` | Nodo que mediante un sistema TTS, convierte texto a voz |
| `checker_node.cpp`| Nodo que realiza la validacion de la postura dependiendo del tiempo de respuesta y la postura a realizar |
| `movement_publisher_node.cpp` | Nodo que publica al action que mueve el robot, la configuracion de los joints del robot|
| `postures.cpp`| Scripts que contiene las condiciones para validar una postura basado en landmarks de Mediapipe|

---


## Preguntas frecuentes

**¿Puedo usar el sistema sin el robot físico?**  
Sí. Funciona completamente en simulación usando Gazebo.

**¿Qué cámara se recomienda?**  
Cualquier cámara USB compatible con `usb_cam`.

**¿Dónde se guardan los reportes?**  
Como archivos `.json`, accesibles desde la aplicación móvil.

