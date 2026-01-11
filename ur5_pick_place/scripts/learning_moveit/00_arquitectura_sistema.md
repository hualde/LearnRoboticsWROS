# Arquitectura del Sistema: Desde el URDF hasta MoveIt 2

Este documento explica cómo se conectan todas las piezas del rompecabezas para que el robot se mueva.

## 1. Descripción del Robot (URDF/Xacro)
Todo empieza con la definición física del robot.
*   **Archivo Principal**: `ur_yt_sim/urdf/ur5_camera_gripper.urdf.xacro`
    *   Este archivo "ensambla" el robot completo importando macros.
*   **Componentes**:
    *   `ur5_macro.xacro`: Brazo UR5 (Cinemática y visuales).
    *   `robotiq_gripper_macro.xacro`: Pinza Robotiq 85.
    *   `camera.xacro`: Cámara (simulada).
*   **Hardware Interface**:
    *   `ur5.ros2_control.xacro`: Define cómo ROS 2 Control habla con los motores del brazo (o con Gazebo).
    *   `robotiq_gripper.ros2_control.xacro`: Lo mismo para la pinza.

## 2. ROS 2 Control
Es la capa intermedia que gestiona los drivers y controladores.
*   **Configuración**: `ur_yt_sim/config/ur5_controllers_gripper.yaml`
*   **Controladores Activos**:
    *   `joint_state_broadcaster`: Publica el estado actual de los joints (`/joint_states`).
    *   `joint_trajectory_controller`: Controla el brazo. Acepta trayectorias completas.
    *   `gripper_position_controller`: Controla la pinza (abrir/cerrar).

## 3. Configuración de MoveIt 2
MoveIt es el "cerebro" que planifica.
*   **Paquete**: `ur5_camera_gripper_moveit_config`
*   **SRDF**: Define los grupos de planificación (`ur5_manipulator` y `gripper`) y las colisiones permitidas.
*   **Joint Limits**: `config/joint_limits.yaml`.
    *   ⚠️ **Importante**: Aquí activamos `has_acceleration_limits: true` para que Pilz funcione.

## 4. Planificación (Planning Pipelines)
MoveIt usa plugins para calcular rutas.
*   **OMPL (Open Motion Planning Library)**: El estándar (RRT, RRTConnect). Bueno para esquivar obstáculos complejos, pero genera caminos "aleatorios".
*   **Pilz Industrial Motion Planner**: El que usamos en `08_pick_place_refined.py`.
    *   **PTP**: Movimiento punto a punto rápido.
    *   **LIN**: Movimiento lineal preciso (Cartesiano).

## 5. Capa de Aplicación (Tus Scripts)
Donde nosotros trabajamos.
*   **Ubicación**: `ur5_pick_place/scripts/learning_moveit/`
*   **Flujo**:
    1.  Tu script (Python) usa `moveit_msgs` para crear un `Goal`.
    2.  Lo envía al Action Server `/move_action`.
    3.  MoveIt planifica usando Pilz/OMPL.
    4.  MoveIt envía la trayectoria al `joint_trajectory_controller`.
    5.  El controlador mueve el robot (Simulación/Real).

---
**Resumen del Flujo de Datos:**
`Python Script` -> `MoveIt 2` -> `ROS 2 Control` -> `Gazebo / Robot Real`
