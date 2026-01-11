# 🤖 Proyecto Pick and Place con Visión - UR5 + Robotiq Gripper

Sistema completo de pick and place guiado por visión para robot UR5 con gripper Robotiq en ROS 2 Humble.

---

## 📋 Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Arquitectura del Sistema](#arquitectura-del-sistema)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
- [Uso](#uso)
- [Componentes](#componentes)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Troubleshooting](#troubleshooting)
- [Desarrollo Futuro](#desarrollo-futuro)

---

## 🎯 Descripción General

Este proyecto implementa un sistema completo de manipulación robótica que integra:

- ✅ **Detección visual de objetos** por color (RGB-D camera)
- ✅ **Transformación de coordenadas** (píxel → 3D → frame del robot)
- ✅ **Planificación de movimientos** con MoveIt 2
- ✅ **Control del brazo robótico** UR5 (6 DOF)
- ✅ **Control del gripper** Robotiq 85
- ✅ **Simulación completa** en Gazebo

### Flujo del Sistema

```
📷 Cámara RGB-D
    ↓
🔍 Detección de objetos (OpenCV)
    ↓
📍 Coordenadas 3D (PointCloud → TF)
    ↓
📡 Publicación en /detected_objects
    ↓
🧠 Planificación con MoveIt
    ↓
🤖 Ejecución en UR5
    ↓
✋ Control del gripper
```

---

## 🏗️ Arquitectura del Sistema

### Hardware (Simulado)
- **Robot:** Universal Robots UR5 (6 DOF)
- **Gripper:** Robotiq 85
- **Sensor:** Cámara RGB-D (tipo Kinect)
- **Entorno:** Gazebo con world2 personalizado

### Software Stack
- **ROS 2:** Humble
- **Simulación:** Gazebo Classic
- **Planificación:** MoveIt 2
- **Visión:** OpenCV + sensor_msgs
- **Control:** ros2_control

### Nodos Principales

| Nodo | Función | Topics Pub/Sub |
|------|---------|----------------|
| `vision_detector` | Detecta objetos y calcula posiciones 3D | Pub: `/detected_objects`, `/object_markers` |
| `robot_mover_action` | Control del brazo con MoveIt | Sub: `/joint_states`, Action: `/move_action` |
| `gripper_controller` | Control del gripper Robotiq | Action: `/gripper_position_controller/gripper_cmd` |
| `pick_place_main` | Coordinador del pipeline completo | Sub: `/detected_objects` |

---

## 📦 Requisitos

### Sistema Operativo
- Ubuntu 22.04 LTS

### ROS 2
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-moveit-ros-perception
```

### Dependencias Python
```bash
# OpenCV y CV Bridge
sudo apt install ros-humble-cv-bridge python3-opencv

# Scipy (para transformaciones)
pip3 install scipy

# Sensor messages
sudo apt install ros-humble-sensor-msgs-py
```

### Paquetes del Proyecto
- `ur_yt_sim` - Simulación y configuración del robot
- `ur5_camera_gripper_moveit_config` - Configuración de MoveIt
- `ur5_pick_place` - Scripts de control y visión

---

## 🚀 Instalación

### 1. Clonar el Repositorio

```bash
cd ~/misCosas/ros2_ws/src
git clone <URL_DEL_REPO> LearnRoboticsWROS
```

### 2. Instalar Dependencias

```bash
cd ~/misCosas/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compilar

```bash
cd ~/misCosas/ros2_ws
colcon build
source install/setup.bash
```

### 4. Verificar Instalación

```bash
# Listar paquetes instalados
ros2 pkg list | grep ur

# Debería mostrar:
# - ur_yt_sim
# - ur5_camera_gripper_moveit_config
# - ur5_pick_place
```

---

## 🎮 Uso

### Lanzar el Sistema Completo

#### Terminal 1: Simulación + MoveIt
```bash
cd ~/misCosas/ros2_ws
source install/setup.bash
ros2 launch ur_yt_sim spawn_ur5_camera_gripper_moveit.launch.py
```

**Esto inicia:**
- ✅ Gazebo con world2
- ✅ Robot UR5 con gripper
- ✅ Cámara RGB-D
- ✅ MoveIt con RViz
- ✅ Controladores (brazo + gripper)

#### Terminal 2: Sistema de Visión
```bash
cd ~/misCosas/ros2_ws/src/LearnRoboticsWROS/ur_yt_sim/scripts
python3 vision_detector.py
```

**Esto publica:**
- `/detected_objects` - JSON con objetos y posiciones 3D
- `/object_poses` - PoseArray para MoveIt
- `/object_markers` - Marcadores para RViz

#### Terminal 3: Control del Robot
```bash
cd ~/misCosas/ros2_ws/src/LearnRoboticsWROS/ur5_pick_place/scripts/API_moveit
python3 robot_mover_action.py
```

**Menú interactivo:**
```
1. Mover a HOME
2. Mover a ZERO
3. Mover a UP
```

---

## 🧩 Componentes

### 1. Sistema de Visión (`vision_detector.py`)

**Funcionalidad:**
- Detecta objetos de colores: rojo, azul, verde, amarillo, naranja
- Obtiene coordenadas 3D desde la imagen de profundidad
- Transforma coordenadas de `camera_link` a `base_link`
- Publica resultados en formato JSON y marcadores 3D

**Configuración:**
```python
# Rangos de color HSV (modificables)
'red': [(0, 100, 100), (10, 255, 255)]
'green': [(40, 100, 100), (80, 255, 255)]
'blue': [(100, 100, 100), (130, 255, 255)]
```

**Topics:**
- `/detected_objects` (String/JSON)
- `/object_poses` (PoseArray)
- `/object_markers` (MarkerArray)

**Ejemplo de salida:**
```json
[
  {
    "id": 0,
    "color": "green",
    "pixel_x": 320,
    "pixel_y": 240,
    "area": 1234.5,
    "position_camera_frame": {"x": 0.456, "y": 0.123, "z": 0.852},
    "position_robot_frame": {"x": 0.523, "y": -0.034, "z": 1.652}
  }
]
```

---

### 2. Control del Robot (`robot_mover_action.py`)

**Funcionalidad:**
- Usa Move Action API (mismo que RViz)
- Mueve a poses predefinidas (home, zero, up)
- Planificación y ejecución con MoveIt
- Control preciso de velocidad y aceleración

**Poses Predefinidas:**
```python
"home": [0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0]
"zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"up": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
```

**Parámetros de Movimiento:**
- `max_velocity_scaling_factor`: 0.1 (10% velocidad máxima)
- `max_acceleration_scaling_factor`: 0.1 (10% aceleración máxima)
- `allowed_planning_time`: 5.0 segundos
- `num_planning_attempts`: 10

---

### 3. Cámara RGB-D

**Especificaciones:**
- **Tipo:** Depth camera (RGB + profundidad)
- **Resolución:** 640x480 píxeles
- **Frecuencia:** 10 Hz
- **Campo de visión:** 62.4° horizontal
- **Rango:** 5cm - 8m
- **Posición:** Fija a 1.2m altura, mirando hacia abajo

**Topics:**
- `/camera/image_raw` - Imagen RGB
- `/camera/depth/image_raw` - Mapa de profundidad
- `/camera/points` - Nube de puntos 3D
- `/camera/camera_info` - Parámetros intrínsecos

**Modificar posición:**
```bash
nano ~/misCosas/ros2_ws/src/LearnRoboticsWROS/ur_yt_sim/urdf/camera.xacro
# Línea 36: <origin xyz="0.5 0 1.2" rpy="0 1.57 0"/>
```

---

### 4. Control del Gripper

**Especificaciones:**
- **Modelo:** Robotiq 85
- **Apertura:** 0 - 85mm
- **Joint principal:** `robotiq_85_left_knuckle_joint`
- **Posiciones:** Abierto (0.0) / Cerrado (0.79)

**Action:**
```bash
/gripper_position_controller/gripper_cmd
```

**Uso desde Python:**
```python
from control_msgs.action import GripperCommand

goal = GripperCommand.Goal()
goal.command.position = 0.0  # Abierto
goal.command.max_effort = 100.0
```

---

### 5. Secuencia de Aprendizaje (`learning_moveit/`)

Para fines educativos y pruebas industriales, se ha incluido una serie de scripts secuenciales en `ur5_pick_place/scripts/learning_moveit/`.

#### **Script Principal: `06_sequential_pick_place.py`**
Este script realiza una secuencia completa de Pick & Place con las siguientes especificaciones actuales:

**Punto de Recogida (PICK):**
- **Posición (XYZ):** `[0.500, 0.000, 0.379]`
- **Orientación (Quaternion xyzw):** `[0.737, -0.675, 0.020, 0.006]`

**Punto de Descarga (PLACE):**
- **Posición (XYZ):** `[0.400, 0.545, 0.515]`
- **Orientación (Quaternion xyzw):** `[0.665, -0.600, 0.310, -0.320]`

---

## 📁 Estructura del Proyecto

```
LearnRoboticsWROS/
├── ur_yt_sim/                           # Paquete principal de simulación
│   ├── launch/
│   │   └── spawn_ur5_camera_gripper_moveit.launch.py
│   ├── urdf/
│   │   ├── ur.urdf.xacro                # URDF principal
│   │   └── camera.xacro                 # Configuración de cámara
│   ├── worlds/
│   │   └── world2.world                 # Mundo con objetos
│   ├── config/
│   │   └── ur5_controllers.yaml         # Configuración controladores
│   └── scripts/
│       └── vision_detector.py           # Sistema de visión
│
├── ur5_camera_gripper_moveit_config/    # Configuración MoveIt
│   ├── config/
│   │   ├── ur.srdf                      # Semántica del robot
│   │   ├── moveit_controllers.yaml     # Controladores MoveIt
│   │   ├── kinematics.yaml              # Cinemática (IK/FK)
│   │   └── moveit.rviz                  # Configuración RViz
│   └── launch/
│       └── moveit.launch.py
│
└── ur5_pick_place/                      # Scripts de control
    └── scripts/
        └── API_moveit/
            ├── robot_mover_action.py    # Control del robot
            └── gripper_controller.py    # Control del gripper (futuro)
```

---

## 🔧 Configuración

### Archivos Clave

#### 1. SRDF - Configuración Semántica
**Ubicación:** `ur5_camera_gripper_moveit_config/config/ur.srdf`

**Planning Groups:**
- `ur5_manipulator` - Brazo (6 DOF)
- `robotiq_gripper` - Gripper

**Poses Predefinidas:**
- `home`, `zero`, `up` (brazo)
- `open`, `close` (gripper)

#### 2. Controllers - Configuración de Control
**Ubicación:** `ur5_camera_gripper_moveit_config/config/moveit_controllers.yaml`

```yaml
joint_trajectory_controller:
  type: FollowJointTrajectory
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint

gripper_position_controller:
  type: GripperCommand
  joints:
    - robotiq_85_left_knuckle_joint
```

#### 3. World - Entorno de Simulación
**Ubicación:** `ur_yt_sim/worlds/world2.world`

**Contiene:**
- Mesa de trabajo (1.2m x 1.2m)
- Objetos de colores para agarrar
- Iluminación
- Física configurada

---

## 🎨 Visualización en RViz

### Agregar Marcadores de Objetos

1. En RViz, clic en **"Add"**
2. Seleccionar **"MarkerArray"**
3. Topic: `/object_markers`
4. ✅ Verás esferas de colores en las posiciones de los objetos

### Ver Imagen de Cámara

1. Clic en **"Add"**
2. Seleccionar **"Image"**
3. Topic: `/camera/image_raw`

### Ver Nube de Puntos

1. Clic en **"Add"**
2. Seleccionar **"PointCloud2"**
3. Topic: `/camera/points`
4. Color Transformer: **RGB8**

---

## 🧪 Testing

### Verificar Cámara

```bash
# Ver topics
ros2 topic list | grep camera

# Ver frecuencia
ros2 topic hz /camera/image_raw

# Ver imagen
ros2 run rqt_image_view rqt_image_view
```

### Verificar Controladores

```bash
# Listar controladores
ros2 control list_controllers

# Ver joint states
ros2 topic echo /joint_states --once
```

### Verificar MoveIt

```bash
# Ver si move_group está activo
ros2 node list | grep move_group

# Ver action servers
ros2 action list
```

### Verificar Transformaciones

```bash
# Ver árbol TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Ver transformación específica
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## 🐛 Troubleshooting

### Problema: "Planning failed! Error code: FAILURE"

**Causa:** MoveIt no puede encontrar una trayectoria válida

**Soluciones:**
1. Verificar que el objetivo está dentro del workspace
2. Probar con `num_planning_attempts` mayor
3. Aumentar `allowed_planning_time`
4. Verificar colisiones en RViz

```python
goal_msg.request.num_planning_attempts = 20
goal_msg.request.allowed_planning_time = 10.0
```

---

### Problema: Cámara no detecta objetos

**Verificaciones:**
```bash
# 1. ¿La cámara publica?
ros2 topic hz /camera/image_raw

# 2. ¿Hay objetos en la escena?
# Verificar en Gazebo

# 3. ¿Los rangos de color son correctos?
# Ajustar HSV en vision_detector.py
```

**Ajustar rangos HSV:**
```python
# vision_detector.py, línea ~65
'green': [(40, 100, 100), (80, 255, 255)]  # Ampliar rango si es necesario
```

---

### Problema: Robot no se mueve desde Python

**Debug:**
```bash
# 1. ¿Funciona desde RViz?
# Probar "Plan & Execute" en RViz

# 2. ¿Los controladores están activos?
ros2 control list_controllers

# 3. ¿El action server responde?
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{}"
```

---

### Problema: Transformaciones TF incorrectas

**Verificar:**
```bash
# Ver frames disponibles
ros2 run tf2_tools view_frames

# Ver transformación
ros2 run tf2_ros tf2_echo base_link camera_link

# Debe mostrar:
# Translation: [0.500, 0.000, 1.200]
```

**Corregir en camera.xacro:**
```xml
<origin xyz="0.5 0 1.2" rpy="0 1.57 0"/>
```

---

### Problema: Gazebo se congela

**Causas comunes:**
- Física muy pesada
- Demasiados objetos
- GPU insuficiente

**Soluciones:**
```bash
# 1. Reducir calidad gráfica
export LIBGL_ALWAYS_SOFTWARE=1

# 2. Pausar Gazebo cuando no se usa
# Clic en "Pause" en Gazebo GUI

# 3. Reducir real_time_update_rate en world
# En world2.world:
<real_time_update_rate>500.0</real_time_update_rate>
```

---

## 📊 Parámetros Ajustables

### Velocidad del Robot

```python
# En robot_mover_action.py
goal_msg.request.max_velocity_scaling_factor = 0.1  # 0.1 = 10%
goal_msg.request.max_acceleration_scaling_factor = 0.1
```

### Detección de Colores

```python
# En vision_detector.py
color_ranges = {
    'green': [(40, 100, 100), (80, 255, 255)]  # [H_min, S_min, V_min], [H_max, S_max, V_max]
}
```

### Frecuencia de Detección

```python
# En vision_detector.py, línea ~45
self.process_rate = 2.0  # Hz (detecciones por segundo)
```

### Cámara

```xml
<!-- En camera.xacro -->
<update_rate>10</update_rate>  <!-- FPS -->
<horizontal_fov>1.089</horizontal_fov>  <!-- Campo de visión en radianes -->
<image><width>640</width><height>480</height></image>  <!-- Resolución -->
```

---

## 🚀 Desarrollo Futuro

### Corto Plazo (Próximos pasos)
- [ ] Implementar secuencia completa de pick
- [ ] Implementar secuencia completa de place
- [ ] Pipeline automático pick and place
- [ ] Control del gripper integrado

### Mediano Plazo
- [ ] Detección con deep learning (YOLO)
- [ ] Estimación de pose 6D de objetos
- [ ] Planificación de trayectorias optimizada
- [ ] Manejo robusto de errores

### Largo Plazo
- [ ] Múltiples objetos en paralelo
- [ ] Clasificación y sorting por color
- [ ] Apilamiento de objetos
- [ ] Aprendizaje por refuerzo

---

## 📚 Referencias

### Documentación Oficial
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html)
- [Gazebo Classic](https://classic.gazebosim.org/)
- [ros2_control](https://control.ros.org/master/index.html)

### Tutoriales Relacionados
- [MoveIt Python API](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html)
- [OpenCV Python](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

---

## 👥 Contribuciones

Sugerencias para contribuir:
1. Fork del repositorio
2. Crear branch (`git checkout -b feature/nueva-funcionalidad`)
3. Commit cambios (`git commit -m 'Agregar nueva funcionalidad'`)
4. Push al branch (`git push origin feature/nueva-funcionalidad`)
5. Abrir Pull Request

---

## 📝 Licencia

Este proyecto es de código abierto bajo licencia [especificar licencia].

---

## 🙏 Agradecimientos

- Universal Robots por documentación del UR5
- Robotiq por especificaciones del gripper
- Comunidad de MoveIt
- Comunidad de ROS 2

---

## 📧 Contacto

Para preguntas o soporte:
- **Email:** [tu email]
- **GitHub:** [tu github]
- **ROS Discourse:** [tu usuario]

---

## 🎓 Créditos

Proyecto desarrollado como parte de aprendizaje en robótica y visión artificial con ROS 2.

**Tecnologías utilizadas:**
- ROS 2 Humble
- MoveIt 2
- Gazebo Classic
- OpenCV
- Python 3.10
- C++ (URDF/xacro)

---

**Última actualización:** Enero 2026  
**Versión:** 1.0.0  
**Estado:** En desarrollo activo

---
#Para no visualizar errores en consola
```bash
ros2 launch ur_yt_sim spawn_ur5_camera_gripper_moveit.launch.py 2>&1 | grep -v "Joint.*mimic.*not found"
```
#Sin octomap
```bash
ros2 launch ur_yt_sim spawn_ur5_camera_gripper_moveit.launch.py with_octomap:=false 2>&1 | grep -v "Joint.*mimic.*not found"
```
#Añadimos colisiones
```bash
ros2 run ur_yt_sim test_ik_collision_obj
```

#Servicio pinza
```bash
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{
  model1_name: 'cobot',
  link1_name: 'wrist_3_link',
  model2_name: 'cube_pick',
  link2_name: 'link_1'
}"
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{
  model1_name: 'cobot',
  link1_name: 'wrist_3_link',
  model2_name: 'cube_pick',
  link2_name: 'link_1'
}"

```

## 🐳 Docker

### Quick Start
```bash
# Pull from Docker Hub
docker pull hualde/ur5-robotiq85:latest

# Allow X11 forwarding
xhost +si:localuser:root

# Run container
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/dri:/dev/dri \
  hualde/ur5-robotiq85:latest

# Inside container, launch simulation
ros2 launch ur_yt_sim spawn_ur5_camera_gripper_moveit.launch.py 2>&1 | grep -v "Joint.*mimic.*not found"

```

### Building from Source
```bash
git clone https://github.com/YOUR_USERNAME/LearnRoboticsWROS.git
cd LearnRoboticsWROS
docker build -t ur5-robotiq85:latest .
```

### Docker Hub
Image available at: https://hub.docker.com/r/hualde/ur5-robotiq85

### What's Included
- ROS2 Humble Desktop Full
- Gazebo simulation
- MoveIt2 motion planning
- UR5 robot with Robotiq 85 gripper
- Camera integration
- Pick and place capabilities

### System Requirements
- Ubuntu 22.04 (or WSL2 on Windows)
- Docker installed
- X11 for GUI (Gazebo/RViz)
- GPU drivers for hardware acceleration
```

Guardar: `Ctrl+O`, `Enter`, `Ctrl+X`

---

## Cuando el build termine exitosamente

Verás algo como:
```
 => exporting to image
 => => exporting layers
 => => writing image sha256:abc123...
 => => naming to docker.io/library/ur5-robotiq85:latest
Successfully built abc123def456
Successfully tagged ur5-robotiq85:latest