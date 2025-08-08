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

### 2. Clonar el repositorio

```bash
cd src
git clone https://github.com/egonzalezauh/Materia-Integradora-Gonzalez-Lucin.git
cd ..
```

### 3. Instalar dependencias del sistema

```bash
sudo apt install python3-colcon-common-extensions
```

### 4. Instalar dependencias ROS 2 del proyecto

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
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

## Estructura del sistema ROS 2

| Paquete | Función |
|--------|---------|
| `posture_game` | Lógica del juego, retroalimentación, estadísticas |
| `checker_node` | Validación de posturas con visión por computadora |
| `mediapipe_node` | Procesamiento de imagen con MediaPipe |
| `detect_emotions_node` | Detección de emociones faciales con IA |
| `movement_controller` | Control de posturas en el robot humanoide |
| `coco_gazebo_sim` | Entorno de simulación en Gazebo |
| `coco_services` | Scripts para iniciar nodos |
| `rosbridge_server` | Comunicación WebSocket con la app móvil |

---

## Preguntas frecuentes

**¿Puedo usar el sistema sin el robot físico?**  
Sí. Funciona completamente en simulación usando Gazebo.

**¿Qué cámara se recomienda?**  
Cualquier cámara USB compatible con `usb_cam`.

**¿Dónde se guardan los reportes?**  
Como archivos `.json`, accesibles desde la aplicación móvil.

