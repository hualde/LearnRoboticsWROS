# 🤖 robot_mover_action.py - Documentación

Control del brazo robótico UR5 usando MoveIt 2 Move Action API.

---

## 📋 Índice

- [Descripción](#descripción)
- [¿Por Qué Esta Implementación?](#por-qué-esta-implementación)
- [Requisitos](#requisitos)
- [Uso](#uso)
- [Arquitectura](#arquitectura)
- [API Reference](#api-reference)
- [Ejemplos de Código](#ejemplos-de-código)
- [Parámetros Configurables](#parámetros-configurables)
- [Troubleshooting](#troubleshooting)
- [Comparación con Otras APIs](#comparación-con-otras-apis)

---

## 🎯 Descripción

`robot_mover_action.py` es un nodo de ROS 2 que controla el brazo robótico UR5 utilizando la **Move Action API** de MoveIt 2. Esta es la misma API que utiliza RViz internamente, lo que garantiza compatibilidad y confiabilidad.

### Características Principales

✅ **Control de joints directo** - Mover a posiciones angulares específicas  
✅ **Poses predefinidas** - Home, zero, up  
✅ **Planificación robusta** - 10 intentos, 5 segundos de tiempo  
✅ **Velocidad controlada** - 10% de velocidad/aceleración máxima  
✅ **Monitoreo de estado** - Feedback en tiempo real  
✅ **Compatible con RViz** - Usa la misma acción que RViz  

---

## 🤔 ¿Por Qué Esta Implementación?

### Problema Inicial

Intentamos usar **pymoveit2** (la API Python de alto nivel), pero fallaba con:
```
[WARN] Planning failed! Error code: FAILURE
```

### Solución

Usar directamente la **Move Action API** (`/move_action`) que es:
- ✅ La que usa RViz internamente
- ✅ Más estable y confiable
- ✅ Mayor control sobre parámetros de planificación
- ✅ Mejor debugging

### Comparación

| API | Ventajas | Desventajas | Estado |
|-----|----------|-------------|--------|
| **pymoveit2** | Fácil de usar | No funcionó en nuestro setup | ❌ Falló |
| **moveit_commander** | API oficial Python | Requiere instalación extra | ⚠️ No probada |
| **Move Action** | Funciona siempre | Más verboso | ✅ **Usamos esta** |

---

## 📦 Requisitos

### Paquetes ROS 2
```bash
sudo apt install ros-humble-moveit-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-rclpy
```

### Dependencias Python
```bash
# Ya incluidas en ROS 2 Humble
# No se necesita instalar nada adicional
```

### Sistema Corriendo

Antes de ejecutar este script, debes tener:

1. **Simulación activa:**
   ```bash
   ros2 launch ur_yt_sim spawn_ur5_camera_gripper_moveit.launch.py
   ```

2. **Verificar que MoveIt está activo:**
   ```bash
   ros2 node list | grep move_group
   # Debe mostrar: /move_group
   ```

3. **Verificar action server:**
   ```bash
   ros2 action list | grep move_action
   # Debe mostrar: /move_action
   ```

---

## 🚀 Uso

### Ejecución Básica

```bash
cd ~/misCosas/ros2_ws/src/LearnRoboticsWROS/ur5_pick_place/scripts/API_moveit
python3 robot_mover_action.py
```

### Menú Interactivo

```
======================================================================
🤖 ROBOT MOVER - Move Action (como RViz)
======================================================================
[INFO] Inicializando con Move Action...
⏳ Esperando action server...
✅ Listo!
✅ Joints actuales: ['0.000', '-2.256', '1.406', '-1.632', '-1.570', '0.000']

📋 MENÚ:
  1. Mover a HOME
  2. Mover a ZERO
  3. Mover a UP
  0. Salir

Opción: _
```

### Salida de Ejemplo

```
Opción: 1

🏠 Moviendo a HOME...
[INFO] 🔧 Moviendo a: ['0.000', '-2.256', '1.406', '-1.632', '-1.570', '0.000']
[INFO] ⚙️  Enviando goal...
[INFO] ✅ Goal aceptado, ejecutando...
[INFO] ✅ Movimiento completado!
```

---

## 🏗️ Arquitectura

### Diagrama de Flujo

```
┌─────────────────────┐
│  robot_mover_action │
│      (Tu script)    │
└──────────┬──────────┘
           │
           │ ActionClient
           ↓
    ┌──────────────┐
    │ /move_action │ ← Action Server de MoveIt
    └──────┬───────┘
           │
           ↓
    ┌──────────────┐
    │  move_group  │ ← Nodo de MoveIt
    └──────┬───────┘
           │
           ↓
    ┌──────────────┐
    │ Controllers  │ ← ros2_control
    └──────┬───────┘
           │
           ↓
    ┌──────────────┐
    │   Robot UR5  │ ← Gazebo/Hardware
    └──────────────┘
```

### Topics y Actions

**Subscripciones:**
- `/joint_states` (sensor_msgs/JointState) - Estado actual del robot

**Action Clients:**
- `/move_action` (moveit_msgs/action/MoveGroup) - Comando de movimiento

**No publica topics** - Solo usa actions

---

## 📚 API Reference

### Clase Principal: `RobotMoverAction`

```python
class RobotMoverAction(Node):
    """
    Nodo para controlar el robot UR5 usando Move Action API
    """
```

#### Constructor

```python
def __init__(self):
    """
    Inicializa el nodo y el action client.
    
    Configura:
    - Action client para /move_action
    - Suscripción a /joint_states
    - Espera a que el action server esté disponible
    """
```

---

#### Métodos Públicos

##### `move_to_joint_state(joint_positions)`

Mueve el robot a posiciones específicas de articulaciones.

**Parámetros:**
- `joint_positions` (list[float]): Lista de 6 ángulos en radianes
  - `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`

**Returns:**
- `bool`: True si el movimiento fue exitoso, False si falló

**Ejemplo:**
```python
robot = RobotMoverAction()

# Mover a HOME
home = [0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0]
success = robot.move_to_joint_state(home)

if success:
    print("✅ Llegó a HOME")
else:
    print("❌ Fallo la planificación")
```

**Proceso Interno:**
1. Verifica que joint_states estén disponibles
2. Crea un `MoveGroup.Goal()` con constraints
3. Configura parámetros de planificación
4. Envía el goal al action server
5. Espera resultado (máx 30 segundos)
6. Retorna success/failure

---

##### `joint_callback(msg)`

Callback interno para `/joint_states`.

**Parámetros:**
- `msg` (sensor_msgs/JointState): Mensaje con estado de todas las articulaciones

**Efecto:**
- Guarda los primeros 6 valores en `self.current_joints`

**Uso:**
```python
# Automático - se llama cuando llegan joint_states
# NO llamar manualmente
```

---

### Atributos de la Clase

```python
self._action_client      # ActionClient para /move_action
self.current_joints      # List[float] - Estado actual de joints (6 valores)
self.joint_sub          # Subscription a /joint_states
```

---

## 💻 Ejemplos de Código

### Ejemplo 1: Uso Básico

```python
#!/usr/bin/env python3
import rclpy
from robot_mover_action import RobotMoverAction

def main():
    rclpy.init()
    
    # Crear nodo
    robot = RobotMoverAction()
    
    # Esperar joint states
    for _ in range(10):
        rclpy.spin_once(robot, timeout_sec=0.5)
        if robot.current_joints is not None:
            break
    
    # Mover a HOME
    home = [0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0]
    robot.move_to_joint_state(home)
    
    # Limpiar
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Ejemplo 2: Secuencia de Movimientos

```python
import time

# Inicializar
robot = RobotMoverAction()

# Poses
home = [0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0]
up = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Secuencia
print("1. Ir a HOME")
robot.move_to_joint_state(home)
time.sleep(2)

print("2. Ir a UP")
robot.move_to_joint_state(up)
time.sleep(2)

print("3. Ir a ZERO")
robot.move_to_joint_state(zero)

print("✅ Secuencia completada")
```

---

### Ejemplo 3: Integración con Visión

```python
from std_msgs.msg import String
import json

class PickAndPlace(RobotMoverAction):
    
    def __init__(self):
        super().__init__()
        
        # Suscribirse a objetos detectados
        self.obj_sub = self.create_subscription(
            String, '/detected_objects', 
            self.object_callback, 10)
    
    def object_callback(self, msg):
        """Recibe objetos detectados"""
        objects = json.loads(msg.data)
        
        if objects:
            obj = objects[0]  # Primer objeto
            
            # Obtener posición
            pos = obj['position_robot_frame']
            x, y, z = pos['x'], pos['y'], pos['z']
            
            print(f"Objeto {obj['color']} en ({x}, {y}, {z})")
            
            # Aquí llamarías a cinemática inversa
            # para convertir (x,y,z) a joint_positions
            # Por ahora, usar HOME como ejemplo
            self.move_to_joint_state([0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0])
```

---

### Ejemplo 4: Con Manejo de Errores

```python
def move_safely(robot, joint_positions, max_retries=3):
    """
    Intenta mover con reintentos
    """
    for attempt in range(max_retries):
        print(f"Intento {attempt + 1}/{max_retries}")
        
        success = robot.move_to_joint_state(joint_positions)
        
        if success:
            print("✅ Éxito")
            return True
        else:
            print("❌ Falló, reintentando...")
            time.sleep(1)
    
    print("💥 Falló después de todos los reintentos")
    return False

# Uso
robot = RobotMoverAction()
home = [0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0]
move_safely(robot, home)
```

---

## ⚙️ Parámetros Configurables

### En el Código

Ubicación: `robot_mover_action.py`, función `move_to_joint_state()`, líneas ~65-72

```python
# Parámetros de planificación
goal_msg.request.group_name = "ur5_manipulator"
goal_msg.request.num_planning_attempts = 10        # ← Cambiar aquí
goal_msg.request.allowed_planning_time = 5.0       # ← Cambiar aquí
goal_msg.request.max_velocity_scaling_factor = 0.1  # ← Cambiar aquí
goal_msg.request.max_acceleration_scaling_factor = 0.1  # ← Cambiar aquí

# Opciones de ejecución
goal_msg.planning_options.plan_only = False  # False = planifica Y ejecuta
goal_msg.planning_options.replan = True      # True = permite replanificar
goal_msg.planning_options.replan_attempts = 5  # ← Cambiar aquí
```

---

### Tabla de Parámetros

| Parámetro | Valor Actual | Rango | Efecto |
|-----------|--------------|-------|--------|
| `num_planning_attempts` | 10 | 1-50 | Más intentos = mayor probabilidad de éxito |
| `allowed_planning_time` | 5.0s | 0.5-30.0s | Más tiempo = mejor plan |
| `max_velocity_scaling` | 0.1 (10%) | 0.01-1.0 | Mayor = más rápido (menos seguro) |
| `max_acceleration_scaling` | 0.1 (10%) | 0.01-1.0 | Mayor = más brusco |
| `replan_attempts` | 5 | 0-20 | Reintentos si falla durante ejecución |

---

### Ejemplos de Ajuste

#### Para Movimientos Rápidos (Menos Seguro)
```python
goal_msg.request.max_velocity_scaling_factor = 0.5  # 50%
goal_msg.request.max_acceleration_scaling_factor = 0.5
goal_msg.request.allowed_planning_time = 2.0  # Menos tiempo
```

#### Para Máxima Seguridad (Más Lento)
```python
goal_msg.request.max_velocity_scaling_factor = 0.05  # 5%
goal_msg.request.max_acceleration_scaling_factor = 0.05
goal_msg.request.num_planning_attempts = 20  # Más intentos
goal_msg.request.allowed_planning_time = 10.0  # Más tiempo
```

#### Para Debugging (Máxima Información)
```python
goal_msg.request.num_planning_attempts = 50
goal_msg.request.allowed_planning_time = 30.0
# + Agregar prints de debug
```

---

### Constraints (Tolerancias)

Ubicación: Líneas ~80-91

```python
constraint = JointConstraint()
constraint.joint_name = name
constraint.position = float(position)
constraint.tolerance_above = 0.01  # ± 0.01 rad = ± 0.57°
constraint.tolerance_below = 0.01
constraint.weight = 1.0  # Importancia relativa
```

**Ajustar tolerancias:**
- **Más estricto:** `0.001` rad (0.057°) - Mayor precisión
- **Más permisivo:** `0.1` rad (5.7°) - Mayor probabilidad de éxito

---

### Timeouts

```python
# Línea ~105: Timeout para aceptación del goal
rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)

# Línea ~116: Timeout para ejecución
rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
```

**Ajustar según necesidad:**
- Movimientos cortos: `timeout_sec=10.0`
- Movimientos largos: `timeout_sec=60.0`
- Debugging: `timeout_sec=None` (sin límite)

---

## 🐛 Troubleshooting

### Problema 1: "Goal rechazado"

**Síntoma:**
```
[ERROR] ❌ Goal rechazado
```

**Causas posibles:**
1. MoveIt no está corriendo
2. Planning group incorrecto
3. Robot en modo de error

**Solución:**
```bash
# Verificar que MoveIt está activo
ros2 node list | grep move_group

# Verificar action server
ros2 action list | grep move_action

# Reiniciar simulación si es necesario
```

---

### Problema 2: "Planning failed"

**Síntoma:**
```
[WARN] Planning failed! Error code: FAILURE
```

**Causas posibles:**
1. Objetivo fuera de alcance
2. Colisión detectada
3. Configuración cinemática incorrecta
4. Timeout muy corto

**Solución:**
```python
# Aumentar intentos y tiempo
goal_msg.request.num_planning_attempts = 20
goal_msg.request.allowed_planning_time = 10.0

# Verificar en RViz que el movimiento es posible
# Probar manualmente "Plan & Execute"
```

---

### Problema 3: "Joint states are not available yet!"

**Síntoma:**
```
[WARN] Joint states are not available yet!
```

**Causa:**
- El script se ejecutó antes de que `/joint_states` empezara a publicar

**Solución:**
```python
# En main(), agregar espera más larga
print("\n⏳ Esperando joint states...")
for i in range(20):  # Aumentar de 10 a 20
    rclpy.spin_once(robot, timeout_sec=0.5)
    if robot.current_joints is not None:
        break
```

---

### Problema 4: Timeout en Ejecución

**Síntoma:**
```
# Script se congela 30 segundos
[ERROR] Timeout esperando resultado
```

**Causa:**
- Movimiento muy lento
- Robot atascado
- Controladores no responden

**Solución:**
```bash
# Verificar controladores
ros2 control list_controllers

# Ver si el robot se mueve en Gazebo
# Si no se mueve, reiniciar simulación
```

---

### Problema 5: Movimientos Muy Lentos

**Síntoma:**
- Robot se mueve pero tarda mucho

**Causa:**
- `max_velocity_scaling_factor` muy bajo (0.1 = 10%)

**Solución:**
```python
# Aumentar velocidad (cuidado en robot real)
goal_msg.request.max_velocity_scaling_factor = 0.3  # 30%
goal_msg.request.max_acceleration_scaling_factor = 0.3
```

---

### Problema 6: "Error code: 99999"

**Síntoma:**
```
[ERROR] ❌ Error: 99999
```

**Causa:**
- Código de error genérico de MoveIt
- Usualmente problema de configuración

**Solución:**
```bash
# Ver logs completos de move_group
ros2 run rqt_console rqt_console

# Filtrar por "move_group"
# Buscar errores específicos
```

---

## 🔬 Debugging Avanzado

### Ver Mensajes del Action

```bash
# En otra terminal mientras ejecutas el script
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
  "{ request: { group_name: 'ur5_manipulator' } }" \
  --feedback
```

### Monitorear Joint States

```bash
# Ver valores en tiempo real
ros2 topic echo /joint_states
```

### Ver Estado de MoveIt

```bash
# Ver planning scene
ros2 topic echo /monitored_planning_scene

# Ver trayectorias planificadas
ros2 topic echo /move_group/display_planned_path
```

---

## 📊 Comparación con Otras APIs

### pymoveit2 (No funcionó en nuestro caso)

```python
# Código equivalente con pymoveit2
from pymoveit2 import MoveIt2

moveit2 = MoveIt2(node=self, joint_names=..., ...)
moveit2.move_to_configuration(joint_positions)
moveit2.wait_until_executed()
```

**Pros:**
- ✅ Código más corto
- ✅ API de alto nivel

**Contras:**
- ❌ Falló en nuestro setup
- ❌ Menos control sobre parámetros
- ❌ Debugging difícil

---

### MoveIt Commander (Requiere instalación)

```python
# Código equivalente con moveit_commander
import moveit_commander

move_group = moveit_commander.MoveGroupCommander("ur5_manipulator")
move_group.set_joint_value_target(joint_positions)
move_group.go(wait=True)
move_group.stop()
```

**Pros:**
- ✅ API oficial de MoveIt
- ✅ Muy usado en tutoriales

**Contras:**
- ❌ Requiere `sudo apt install ros-humble-moveit-commander`
- ⚠️ No lo probamos

---

### Move Action (Nuestra implementación actual)

```python
# Código con Move Action
goal_msg = MoveGroup.Goal()
goal_msg.request.group_name = "ur5_manipulator"
# ... configurar constraints ...
self._action_client.send_goal_async(goal_msg)
```

**Pros:**
- ✅ **Funciona siempre**
- ✅ Mismo que usa RViz
- ✅ Control total sobre parámetros
- ✅ Fácil debugging

**Contras:**
- ❌ Más líneas de código
- ❌ Requiere entender el mensaje `MoveGroup.Goal`

---

## 📈 Rendimiento

### Tiempos Típicos

| Operación | Tiempo |
|-----------|--------|
| Inicialización del nodo | ~1 segundo |
| Espera de action server | ~0.5 segundos |
| Planificación (éxito) | 0.5-2 segundos |
| Planificación (fallo) | 5 segundos (timeout) |
| Ejecución home→zero | ~8 segundos |
| Ejecución zero→up | ~5 segundos |

### Optimizaciones Posibles

1. **Planificador más rápido:**
   ```python
   # En ur5_camera_gripper_moveit_config/config/ompl_planning.yaml
   # Cambiar de RRTConnect a RRT
   ```

2. **Menos intentos si es urgente:**
   ```python
   goal_msg.request.num_planning_attempts = 5  # En vez de 10
   goal_msg.request.allowed_planning_time = 2.0  # En vez de 5.0
   ```

3. **Mayor velocidad (solo simulación):**
   ```python
   goal_msg.request.max_velocity_scaling_factor = 1.0  # 100%
   ```

---

## 🎓 Recursos Adicionales

### Documentación Oficial
- [MoveIt 2 Actions](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [moveit_msgs/MoveGroup](https://github.com/ros-planning/moveit_msgs/blob/ros2/action/MoveGroup.action)
- [ROS 2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

### Tutoriales Relacionados
- [MoveIt 2 Python Tutorial](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html)
- [ros2_control Tutorial](https://control.ros.org/master/doc/ros2_control/doc/index.html)

### Código de Ejemplo
- [MoveIt Examples Repository](https://github.com/ros-planning/moveit2_tutorials)

---

## 🔄 Versiones

| Versión | Fecha | Cambios |
|---------|-------|---------|
| 1.0.0 | Enero 2026 | Implementación inicial funcional |
| 1.1.0 | TBD | Agregar control cartesiano |
| 2.0.0 | TBD | Integrar con pick and place completo |

---

## 📝 Notas de Desarrollo

### Por Qué Este Enfoque

Después de probar múltiples APIs, esta implementación con Move Action directa fue la única que funcionó de manera confiable con nuestra configuración específica de:
- ROS 2 Humble
- MoveIt 2
- UR5 simulado
- ros2_control con Gazebo

### Lecciones Aprendidas

1. **pymoveit2 no siempre funciona** - Depende mucho de la configuración
2. **RViz es tu amigo** - Si funciona en RViz, usa la misma API
3. **Los actions son robustos** - API de bajo nivel pero confiable
4. **Parámetros importan** - Velocity scaling y planning time son críticos

---

## 🤝 Contribuciones

Para mejorar este script:
1. Agregar control cartesiano (pose target)
2. Implementar cinemática inversa
3. Agregar movimientos relativos
4. Mejorar manejo de errores
5. Agregar logging configurable

---

**Autor:** [Tu nombre]  
**Última actualización:** Enero 2026  
**Versión:** 1.0.0  
**Licencia:** [Especificar]